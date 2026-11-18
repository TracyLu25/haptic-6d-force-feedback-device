using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using TMPro;

namespace Unity.Robotics.UrdfImporter.Control
{
    /// <summary>
    /// Nhận va chạm từ Collider trên các Link ABB, ước lượng ngoại lực,
    /// tính moment/dòng điện theo mô hình Delta và gửi dữ liệu về Controller.
    ///
    /// Lưu ý: Đây là lớp tính toán/mô phỏng trong Unity. Không tự tạo lực
    /// vật lý ngược lên robot ABB; dòng điện chỉ có tác dụng phần cứng khi
    /// Controller đang chạy Hardware Mode và STM32 được kết nối.
    /// </summary>
    public class CollisionForceLogger : MonoBehaviour
    {
        // Static counter dùng để throttle debug log trong các hàm static.
        private static int s_debugFrameCounter = 0;
        private static bool s_enableStaticDebug = true;
        private const int DEBUG_LOG_EVERY_N_FRAMES = 30; // ~0.5s ở 60FPS

        [Header("References")]
        [Tooltip("TextMeshPro hiển thị ngoại lực. Có thể để trống, khi đó chỉ ghi log.")]
        public TextMeshProUGUI infoText;

        [Header("Collision Debug")]
        [Tooltip("Ghi log khi Collider bắt đầu va chạm.")]
        public bool logCollisionEvents = true;

        [Tooltip("Ghi giá trị lực trong Console. Có thể tạo nhiều log khi đang tiếp xúc.")]
        public bool logForceToConsole = false;

        [Tooltip("Chỉ log lực mỗi khoảng thời gian này (giây).")]
        [Min(0.05f)]
        public float forceLogInterval = 0.25f;

        [Header("Pipeline Debug")]
        [Tooltip("Log toàn bộ pipeline mỗi khoảng forceLogInterval. Bật = nhiều log.")]
        public bool logPipelineDebug = true;

        [Header("Haptic Calculation")]
        [Tooltip("Bật bù moment trọng lực. Phím M bật, N tắt.")]
        public bool momentCompensate = true;

        // Thông số hình học giữ theo code gốc của hệ thống.
        private const double l_A = 100.0;

        private Controller controller;
        private double[] theta_1i;
        private double[] EEPos;

        // Lực va chạm theo hệ tọa độ Unity, ước lượng từ impulse / fixedDeltaTime.
        private Vector3 forceUnity = Vector3.zero;

        // Các đại lượng sau phép biến đổi hệ tọa độ theo code gốc.
        private Vector3 forceModified = Vector3.zero;
        private Vector3 moments = Vector3.zero;
        private Vector3 current = Vector3.zero;

        private bool hasController;
        private bool warnedMissingText;
        private float nextForceLogTime;

        private void Start()
        {
            // Script thường được gắn trên GameObject Collisions/Link.
            // Tìm Controller trên các GameObject cha.
            controller = GetComponentInParent<Controller>();

            if (controller == null)
            {
                Debug.LogError(
                    "[CollisionForceLogger] Không tìm thấy Controller ở parent của " +
                    gameObject.name + ". Hãy kiểm tra vị trí gắn script.",
                    this
                );
                hasController = false;
                return;
            }

            RefreshControllerArrays();

            if (!hasController)
            {
                Debug.LogError(
                    "[CollisionForceLogger] Controller đã tìm thấy nhưng effectorPos " +
                    "hoặc motorangles chưa được khởi tạo/không đủ 3 phần tử.",
                    this
                );
            }

            if (infoText == null)
            {
                Debug.LogWarning(
                    "[CollisionForceLogger] Chưa gán Info Text. " +
                    "Lực vẫn được tính nhưng không hiển thị trên UI.",
                    this
                );
                warnedMissingText = true;
            }

            nextForceLogTime = Time.time;
        }

        private void RefreshControllerArrays()
        {
            if (controller == null)
            {
                hasController = false;
                return;
            }

            // Đọc lại reference để không phụ thuộc vào việc mảng có bị cấp phát lại.
            EEPos = controller.effectorPos;
            theta_1i = controller.motorangles;

            hasController =
                EEPos != null && EEPos.Length >= 3 &&
                theta_1i != null && theta_1i.Length >= 3;
        }

        private void Update()
        {
            if (controller == null)
                controller = GetComponentInParent<Controller>();

            if (!hasController)
                RefreshControllerArrays();

            if (!hasController)
            {
                UpdateForceText(Vector3.zero);
                return;
            }

            // Đọc vị trí đầu công tác và góc 3 motor Delta từ Controller.
            Vector3 eePosition = new Vector3(
                (float)EEPos[0],
                (float)EEPos[1],
                (float)EEPos[2]
            );

            // Giữ nguyên phép đổi trục/tỉ lệ của hệ thống cũ.
            forceModified = new Vector3(
                forceUnity.z / 60.0f,
                -forceUnity.x / 48.0f,
                forceUnity.y / 10.0f
            );

            // Phím M: bật bù trọng lực; phím N: tắt bù trọng lực.
            if (Input.GetKeyDown(KeyCode.M))
                momentCompensate = true;

            if (Input.GetKeyDown(KeyCode.N))
                momentCompensate = false;

            // Tính moment và dòng điện theo mô hình Delta gốc.
            moments = ComputeMoments(
                eePosition,
                theta_1i[0],
                theta_1i[1],
                theta_1i[2],
                forceModified,
                l_A,
                momentCompensate
            );

            current = ComputeCurrent(moments);

            // Không gửi giá trị NaN/Infinity đi tiếp.
            if (!IsFinite(moments) || !IsFinite(current))
            {
                moments = Vector3.zero;
                current = Vector3.zero;
            }

            // Controller có thể nhận các message này nếu có các hàm tương ứng.
            // SendMessageUpwards giữ cách liên kết của project cũ.
            SendMessageUpwards(
                "ReceiveCurrent",
                current,
                SendMessageOptions.DontRequireReceiver
            );

            SendMessageUpwards(
                "ReceiveForce",
                forceModified,
                SendMessageOptions.DontRequireReceiver
            );

            SendMessageUpwards(
                "ReceiveMoments",
                moments,
                SendMessageOptions.DontRequireReceiver
            );

            UpdateForceText(forceModified);

            if (logForceToConsole && Time.time >= nextForceLogTime)
            {
                Debug.Log(
                    $"[SIM][FORCE] {gameObject.name} | " +
                    $"dt={Time.fixedDeltaTime:F4} | " +
                    $"Unity=({forceUnity.x:F3},{forceUnity.y:F3},{forceUnity.z:F3})"
                );
                Debug.Log(
                    $"[SIM][MAPPING] mapped=({forceModified.x:F3},{forceModified.y:F3},{forceModified.z:F3})"
                );
                Debug.Log(
                    $"[SIM][TORQUE] tau=({moments.x:F4},{moments.y:F4},{moments.z:F4})"
                );
                Debug.Log(
                    $"[SIM][GRAVITY] enabled={momentCompensate}"
                );
                Debug.Log(
                    $"[SIM][CURRENT] I=({current.x:F4},{current.y:F4},{current.z:F4}) unit=mA(calc)"
                );

                nextForceLogTime = Time.time + Mathf.Max(0.05f, forceLogInterval);
            }
        }

        private void UpdateForceText(Vector3 value)
        {
            if (infoText == null)
            {
                if (!warnedMissingText)
                {
                    Debug.LogWarning(
                        "[CollisionForceLogger] Info Text chưa được gán trong Inspector.",
                        this
                    );
                    warnedMissingText = true;
                }
                return;
            }

            infoText.text =
                $"Force X: {value.x:F2}\n" +
                $"Force Y: {value.y:F2}\n" +
                $"Force Z: {value.z:F2}";
        }

        // Được Unity gọi khi bắt đầu tiếp xúc.
        private void OnCollisionEnter(Collision collision)
        {
            UpdateCollisionForce(collision);

            if (logCollisionEvents && collision != null)
            {
                Debug.Log(
                    $"[HAPTIC COLLISION] Enter: {gameObject.name} <-> " +
                    $"{collision.gameObject.name}",
                    this
                );
            }
        }

        // Được Unity gọi trong các bước vật lý khi vẫn đang tiếp xúc.
        private void OnCollisionStay(Collision collision)
        {
            UpdateCollisionForce(collision);
        }

        // Xóa lực khi không còn tiếp xúc.
        private void OnCollisionExit(Collision collision)
        {
            forceUnity = Vector3.zero;

            if (logCollisionEvents && collision != null)
            {
                Debug.Log(
                    $"[HAPTIC COLLISION] Exit: {gameObject.name} <-> " +
                    $"{collision.gameObject.name}",
                    this
                );
            }
        }

        private void UpdateCollisionForce(Collision collision)
        {
            if (collision == null)
                return;

            float dt = Time.fixedDeltaTime;
            if (dt <= Mathf.Epsilon)
            {
                forceUnity = Vector3.zero;
                return;
            }

            // Ước lượng lực trung bình từ impulse trong một physics timestep.
            forceUnity = collision.impulse / dt;

            if (logPipelineDebug)
            {
                Debug.Log(
                    $"[SIM][COLLISION] frame={Time.frameCount} " +
                    $"contact={collision.gameObject.name} " +
                    $"impulse=({collision.impulse.x:F3},{collision.impulse.y:F3},{collision.impulse.z:F3}) " +
                    $"dt={dt:F4}"
                );
            }

            if (!IsFinite(forceUnity))
                forceUnity = Vector3.zero;
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        public static Vector3 ComputeCurrent(
            Vector3 Moment
            )
        {
            Vector3 Current;
            /*
            Current.x = (float)(((Moment.x/1000) / 10) / 0.00022)*1000; // (Moment/1000) là từ N.mm sang N.m. Sau đó /10 là để đổi từ moment trên đĩa Capstan xuống động cơ
            Current.y = (float)(((Moment.y/1000) / 10) / 0.00037)*1000; // 0.07 là K_t
            Current.z = (float)(((Moment.z / 1000) / 10) / 0.00028)*1000; // * 1000 là để đổi từ A sang mA
            */
            // Tính toán với chai nước 0.5kg
            Current.x = (float)(((Moment.x) / 10) / 0.074); // (Moment/1000) là từ N.mm sang N.m. Sau đó /10 là để đổi từ moment trên đĩa Capstan xuống động cơ
            Current.y = (float)(((Moment.y) / 10) / 0.087); // 0.07 là K_t
            Current.z = (float)(((Moment.z) / 10) / 0.086); // * 1000 là để đổi từ A sang mA
            return Current;
        }

        // Từ ngoại lực tác động lên Robot ---> Tính toán ra moment trên 3 đĩa Capstan 
        public static Vector3 ComputeMoments(
            Vector3 effectorPos,          // target x,y,z
            double theta11, double theta12, double theta13,  // motor angles
            Vector3 forceF,               // applied force (Fx,Fy,Fz)
            double l_A,
            bool moment_compensate
        )
        {
            // Bi
            Vector3 B1 = Bi(theta11);
            Vector3 B2 = Bi(theta12);
            Vector3 B3 = Bi(theta13);
            // Pi
            Vector3 P1 = Pi(effectorPos.x, effectorPos.y, effectorPos.z, 0.0);
            Vector3 P2 = Pi(effectorPos.x, effectorPos.y, effectorPos.z, -120.0);
            Vector3 P3 = Pi(effectorPos.x, effectorPos.y, effectorPos.z, 120.0);
            //Debug.Log("Pi " + P1);
            // Ci
            Vector3 C1 = Ci(P1.x, P1.y, P1.z);
            Vector3 C2 = Ci(P2.x, P2.y, P2.z);
            Vector3 C3 = Ci(P3.x, P3.y, P3.z);
            //Debug.Log("Ci " + P1);
            // Theta values
            double th31 = Theta3i(C1.y);
            double th32 = Theta3i(C2.y);
            double th33 = Theta3i(C3.y);
            double th21 = Theta2i(B1.x, C1.x, th31, theta11);
            double th22 = Theta2i(B2.x, C2.x, th32, theta12);
            double th23 = Theta2i(B3.x, C3.x, th33, theta13);

            bool shouldLog = s_enableStaticDebug && (s_debugFrameCounter % DEBUG_LOG_EVERY_N_FRAMES == 0);

            if (shouldLog)
            {
                Debug.Log(
                    $"[SIM][DELTA DEBUG] " +
                    $"th31={th31:F4}, th32={th32:F4}, th33={th33:F4} | " +
                    $"th21={th21:F4}, th22={th22:F4}, th23={th23:F4}"
                );
            }

            // Nếu tọa độ/góc nằm ngoài workspace Delta, không tính Jacobian
            // bằng các giá trị NaN/Infinity.
            if (!IsFinite(th31) || !IsFinite(th32) || !IsFinite(th33) ||
                !IsFinite(th21) || !IsFinite(th22) || !IsFinite(th23) ||
                !IsFinite(forceF.x) || !IsFinite(forceF.y) || !IsFinite(forceF.z))
                return Vector3.zero;

            //Debug.Log("TH31 " + th31);
            // J_theta diagonal
            double[,] Jth = new double[3, 3];
            double rad21 = th21 * Math.PI / 180.0;
            double rad22 = th22 * Math.PI / 180.0;
            double rad23 = th23 * Math.PI / 180.0;
            Jth[0, 0] = l_A * Math.Sin(rad21) * Math.Sin(th31 * Math.PI / 180.0);
            Jth[1, 1] = l_A * Math.Sin(rad22) * Math.Sin(th32 * Math.PI / 180.0);
            Jth[2, 2] = l_A * Math.Sin(rad23) * Math.Sin(th33 * Math.PI / 180.0);
            // J_v
            Vector3 r1 = Ji_xyz(th31, th21, theta11, 0.0);
            Vector3 r2 = Ji_xyz(th32, th22, theta12, -120.0);
            Vector3 r3 = Ji_xyz(th33, th23, theta13, 120.0);
            double[,] Jv = new double[3, 3] {
            {r1.x, r1.y, r1.z},
            {r2.x, r2.y, r2.z},
            {r3.x, r3.y, r3.z}
        };
            /*
            // Multiply Jv * Jth    
            double[,] M1 = Multiply3x3(Jv, Jth);
            // Invert
            double[,] J = Invert3x3(M1);
            */
            // Invert
            double[,] Jv_inv = Invert3x3(Jv);
            // Multiply Jv_inv * Jth    
            double[,] J = Multiply3x3(Jv_inv, Jth);
            // Transpose
            double[,] JT = Transpose3x3(J);

            if (shouldLog)
            {
                Debug.Log(
                    $"[SIM][JACOBIAN] Jth_diag=({Jth[0,0]:E4},{Jth[1,1]:E4},{Jth[2,2]:E4}) | " +
                    $"Jv_row0=({Jv[0,0]:F4},{Jv[0,1]:F4},{Jv[0,2]:F4})"
                );
            }

            s_debugFrameCounter++;
            //Moment bù trọng lượng của từng motor
            /*
            float motor1 =(float) ComputeTorque_V1(theta11, th21, th31);
            float motor2 = (float)ComputeTorque_V1(theta12, th22, th32);
            float motor3 = (float)ComputeTorque_V1(theta13, th23, th33);
            */


            // Có bù trọng lượng
            float motor1 = (float)ComputeTorque_V2(theta11);
            float motor2 = (float)ComputeTorque_V2(theta12);
            float motor3 = (float)ComputeTorque_V2(theta13);

            /*
            // Không bù trọng lượng 
            motor1 = 0.0f;
             motor2 = 0.0f;
            motor3 = 0.0f;
            */
            // M = JT * F
            Vector3 M;

            if (moment_compensate)
            {
                M.x = (float)(JT[0, 0] * forceF.x + JT[0, 1] * forceF.y + JT[0, 2] * forceF.z) + motor1;
                M.y = (float)(JT[1, 0] * forceF.x + JT[1, 1] * forceF.y + JT[1, 2] * forceF.z) + motor2;
                M.z = (float)(JT[2, 0] * forceF.x + JT[2, 1] * forceF.y + JT[2, 2] * forceF.z) + motor3;
            }

            else
            {
                M.x = (float)(JT[0, 0] * forceF.x + JT[0, 1] * forceF.y + JT[0, 2] * forceF.z);
                M.y = (float)(JT[1, 0] * forceF.x + JT[1, 1] * forceF.y + JT[1, 2] * forceF.z);
                M.z = (float)(JT[2, 0] * forceF.x + JT[2, 1] * forceF.y + JT[2, 2] * forceF.z);
            }

            return M;
        }
        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        // Cân bằng trọng lượng
        private static double DegToRad(double degrees)
        {
            return degrees * Math.PI / 180.0;
        }
        public static double ComputeTorque_V2(double theta1Deg)
        {
            // Kích thước mô hình (đơn vị: mét)
            double l_A = 0.1;

            // Khối lượng (kg)
            double m_B = 0.125 * 2;   // cánh tay bị động, 2 thanh song song
            double m_EE = 0.07;      // đầu công tác

            // Gia tốc trọng trường
            double g = 9.81;

            // Chuyển độ sang radian
            double theta1 = DegToRad(theta1Deg);
            // Tính torque
            double totalTorque = ((m_B + m_EE) * g * (l_A * Math.Cos(theta1))) * 1000;

            return totalTorque;
        }
        public static double ComputeTorque_V1(double theta1Deg, double theta2Deg, double theta3Deg)
        {
            // Kích thước mô hình (đơn vị: mét)
            double l_A = 0.1;
            double l_B = 0.235 * Math.Sin(DegToRad(theta3Deg));
            double r = 0.045;

            // Khối lượng (kg)
            double m_B = 0.5 * 2;   // cánh tay bị động, 2 thanh song song
            double m_EE = 0.6;      // đầu công tác

            // Gia tốc trọng trường
            double g = 9.81;

            // Chuyển độ sang radian
            double theta1 = DegToRad(theta1Deg);
            double theta2 = DegToRad(theta2Deg);

            // Tính torque
            double torque_B = -m_B * g * (l_A * Math.Cos(theta1) + l_B * Math.Cos(theta1 + theta2));
            double torque_EE = -m_EE * g * (l_A * Math.Cos(theta1) + l_B * Math.Cos(theta1 + theta2) + r * Math.Cos(DegToRad(180)));

            double totalTorque = torque_B + torque_EE;

            return totalTorque;
        }
        // Tính Bi
        public static Vector3 Bi(double theta1iDeg)
        {
            // Parameters (match MATLAB constants)
            const double R = 100.0;   // R_A in mm or unit
            const double lA = 100.0;  // l_A in mm or unit

            // Convert degrees to radians
            double thetaRad = theta1iDeg * Math.PI / 180.0;

            // Compute Bi
            double xBi = R + lA * Math.Cos(thetaRad);
            double yBi = 0.0;
            double zBi = lA * Math.Sin(thetaRad);

            // Return as Vector3 (cast to float for Unity)
            return new Vector3((float)xBi, (float)yBi, (float)zBi);
        }

        public static Vector3 Pi(double x, double y, double z, double alphaDeg)
        {
            // Convert degrees to radians
            double alphaRad = alphaDeg * Math.PI / 180.0;

            // Rotation matrix about Z-axis
            double cosA = Math.Cos(alphaRad);
            double sinA = Math.Sin(alphaRad);

            // Apply rotation
            double xPi = cosA * x - sinA * y;
            double yPi = sinA * x + cosA * y;
            double zPi = z; // unchanged

            return new Vector3((float)xPi, (float)yPi, (float)zPi);
        }


        public static Vector3 Ci(double xPi, double yPi, double zPi)
        {
            const double r = 44.55; // r in mm or unit

            double xCi = xPi + r;
            double yCi = yPi;
            double zCi = zPi;

            return new Vector3((float)xCi, (float)yCi, (float)zCi);
        }

        public static double Theta3i(double yCi)
        {
            const double lB = 235.0;
            double ratio = yCi / lB;

            // Không cho Math.Acos nhận giá trị ngoài [-1, 1].
            if (double.IsNaN(ratio) || double.IsInfinity(ratio) ||
                ratio < -1.0 || ratio > 1.0)
                return double.NaN;

            return Math.Acos(ratio) * 180.0 / Math.PI;
        }


        public static double Theta2i(double xBi, double xCi, double theta3Deg, double theta1Deg)
        {
            const double lB = 235.0;
            // Convert theta3 to radians for sin
            double theta3Rad = theta3Deg * Math.PI / 180.0;
            double numerator = xBi - xCi;
            double denominator = lB * Math.Sin(theta3Rad);

            if (Math.Abs(denominator) < 1e-9)
                return double.NaN;

            double ratio = numerator / denominator;
            if (double.IsNaN(ratio) || double.IsInfinity(ratio) ||
                ratio < -1.0 || ratio > 1.0)
                return double.NaN;

            double thetaRad = Math.Acos(ratio);
            return 180.0 - theta1Deg - (thetaRad * 180.0 / Math.PI);
        }

        public static Vector3 Ji_xyz(double theta3Deg, double theta2Deg, double theta1Deg, double alphaDeg)
        {
            // Convert to radians
            double t3 = theta3Deg * Math.PI / 180.0;
            double t2 = theta2Deg * Math.PI / 180.0;
            double t1 = theta1Deg * Math.PI / 180.0;
            double a = alphaDeg * Math.PI / 180.0;

            double sum12 = t1 + t2;
            double sinT3 = Math.Sin(t3);
            double cosT3 = Math.Cos(t3);
            double cosSum = Math.Cos(sum12);
            double sinSum = Math.Sin(sum12);
            double cosA = Math.Cos(a);
            double sinA = Math.Sin(a);

            double Jix = sinT3 * cosSum * cosA + cosT3 * sinA;
            double Jiy = -sinT3 * cosSum * sinA + cosT3 * cosA;
            double Jiz = sinT3 * sinSum;

            return new Vector3((float)Jix, (float)Jiy, (float)Jiz);
        }

        // Helper: multiply 3x3 matrices
        private static double[,] Multiply3x3(double[,] A, double[,] B)
        {
            double[,] C = new double[3, 3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
            {
                double sum = 0;
                for (int k = 0; k < 3; k++) sum += A[i, k] * B[k, j];
                C[i, j] = sum;
            }
            return C;
        }

        // Helper: invert 3x3 matrix
        private static double[,] Invert3x3(double[,] m)
        {
            double a = m[0, 0], b = m[0, 1], c = m[0, 2];
            double d = m[1, 0], e = m[1, 1], f = m[1, 2];
            double g = m[2, 0], h = m[2, 1], i = m[2, 2];
            double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

            if (Math.Abs(det) < 1e-6)
            {
                Debug.Log($"[SIM][JACOBIAN] Singularity: det={det:E6}");
                return MultiplyScalar(Identity3(), 0);
            }
            double invDet = 1.0 / det;
            double[,] inv = new double[3, 3];
            inv[0, 0] = (e * i - f * h) * invDet;
            inv[0, 1] = (c * h - b * i) * invDet;
            inv[0, 2] = (b * f - c * e) * invDet;
            inv[1, 0] = (f * g - d * i) * invDet;
            inv[1, 1] = (a * i - c * g) * invDet;
            inv[1, 2] = (c * d - a * f) * invDet;
            inv[2, 0] = (d * h - e * g) * invDet;
            inv[2, 1] = (b * g - a * h) * invDet;
            inv[2, 2] = (a * e - b * d) * invDet;
            return inv;
        }

        private static double[,] Transpose3x3(double[,] m)
        {
            return new double[,] {
            {m[0,0], m[1,0], m[2,0]},
            {m[0,1], m[1,1], m[2,1]},
            {m[0,2], m[1,2], m[2,2]}
        };
        }

        private static double[,] Identity3()
        {
            return new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        }

        private static double[,] MultiplyScalar(double[,] m, double s)
        {
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) m[r, c] *= s;
            return m;
        }

    }
}
