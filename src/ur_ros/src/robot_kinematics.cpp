#include <ur_ros/robot_kinematics.hpp>

namespace ur_ros
{
    Eigen::Matrix<double, 3, 1> rotMToeulerZYZ(Eigen::Matrix<double, 3, 3> R)
    {
        Eigen::Matrix<double, 3, 1> res;

        // retrieve the settings for a particular axis sequence
        Eigen::Matrix<double, 4, 1> nextAxis;
        nextAxis << 1, 2, 0, 1;

        int firstAxis = 2;
        bool repetition = true;
        int parity = 1;
        bool movingFrame = true;

        // calculate indices for accessing rotation matrix
        int i = firstAxis;
        int j = nextAxis(i + parity);
        int k = nextAxis(i - parity + 1);

        if (repetition)
        {
            // find special cases of rotation matrix values that correspond to Euler
            // angle singularities
            double sy = sqrt(R(i, j) * R(i, j) + R(i, k) * R(i, k));
            bool singular = sy < 10 * 1e-16;

            // calculate Euler angles
            res << atan2(R(i, j), R(i, k)), atan2(sy, R(i, i)), atan2(R(j, i), -R(k, i));

            // singular matrices need special treatment
            if (singular)
            {
                res << atan2(-R(j, k), R(j, j)), atan2(sy, R(i, i)), 0;
            }
        }
        else
        {
            // find special cases of rotation matrix values that correspond to Euler
            // angle singularities
            double sy = sqrt(R(i, i) * R(i, i) + R(j, i) * R(j, i));
            bool singular = sy < 10 * 1e-16;

            // calculate Euler angles
            res << atan2(R(k, j), R(k, k)), atan2(-R(k, i), sy), atan2(R(j, i), R(i, i));

            // singular matrices need special treatment
            if (singular)
            {
                res << atan2(-R(j, k), R(j, j)), atan2(-R(k, i), sy), 0;
            }
        }

        if (parity)
        {
            // invert the result
            res = -res;
        }

        if (movingFrame)
        {
            // swap the X and Z columns
            auto tmp = res[0];
            res[0] = res[2];
            res[2] = tmp;
        }

        return res;
    }

    // the quaternion order is XYZW
    Eigen::Matrix<double, 3, 1> quatToeulerZYZ(Eigen::Matrix<double, 4, 1> q)
    {
        Eigen::Matrix<double, 3, 3> R = Eigen::Quaterniond(q(3), q(2), q(1), q(0)).toRotationMatrix();
        return rotMToeulerZYZ(R);
    }

    Eigen::Matrix<double, 3, 3> eulerZYZToRotM(Eigen::Matrix<double, 3, 1> euler)
    {
        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();

        double ct1 = cos(euler[0]);
        double ct2 = cos(euler[1]);
        double ct3 = cos(euler[2]);
        double st1 = sin(euler[0]);
        double st2 = sin(euler[1]);
        double st3 = sin(euler[2]);

        R(0, 0) = ct1 * ct3 * ct2 - st1 * st3;
        R(0, 1) = -ct1 * ct2 * st3 - st1 * ct3;
        R(0, 2) = ct1 * st2;
        R(1, 0) = st1 * ct3 * ct2 + ct1 * st3;
        R(1, 1) = -st1 * ct2 * st3 + ct1 * ct3;
        R(1, 2) = st1 * st2;
        R(2, 0) = -st2 * ct3;
        R(2, 1) = st2 * st3;
        R(2, 2) = ct2;

        return R;
    }

    Eigen::Matrix<double, 4, 4> getTransformation(int from, int to, Eigen::Matrix<double, dof, 1> q)
    {
        Eigen::Matrix<double, 4, 4> T, tmp;
        T = Eigen::Matrix<double, 4, 4>::Identity();

        for (int i = from; i < to; i++)
        {
            auto ct = cos(q(i));
            auto st = sin(q(i));
            auto ca = cos(dh_alpha[i]);
            auto sa = sin(dh_alpha[i]);

            tmp << ct, -st * ca, st * sa, dh_a[i] * ct,
                st, ct * ca, -ct * sa, dh_a[i] * st,
                0, sa, ca, dh_d[i],
                0, 0, 0, 1;

            T = T * tmp;
        }

        return T;
    }

    Eigen::Matrix<double, 6, 1> forwardKinematics(Eigen::Matrix<double, dof, 1> q)
    {
        Eigen::Matrix<double, 4, 4> T = getTransformation(0, dof, q);
        Eigen::Matrix<double, 3, 3> R = T.block(0, 0, 3, 3);

        Eigen::Matrix<double, 6, 1> res;
        res.head(3) = T.block(0, 3, 3, 1);
        res.segment(3, 3) = rotMToeulerZYZ(R);

        return res;
    }

    int SIGN(double x)
    {
        return (x > 0) - (x < 0);
    }

    Eigen::Matrix4d AH(int n, const Eigen::MatrixXd &th, int c, const Eigen::VectorXd &a, const Eigen::VectorXd &d, const Eigen::VectorXd &alpha)
    {
        Eigen::Matrix4d T_a = Eigen::Matrix4d::Identity();
        T_a(0, 3) = a(n);
        Eigen::Matrix4d T_d = Eigen::Matrix4d::Identity();
        T_d(2, 3) = d(n);

        Eigen::Matrix4d Rzt;
        Rzt << cos(th(n, c)), -sin(th(n, c)), 0, 0,
            sin(th(n, c)), cos(th(n, c)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        Eigen::Matrix4d Rxa;
        Rxa << 1, 0, 0, 0,
            0, cos(alpha(n)), -sin(alpha(n)), 0,
            0, sin(alpha(n)), cos(alpha(n)), 0,
            0, 0, 0, 1;

        return T_d * Rzt * T_a * Rxa;
    }

    Eigen::Matrix4d HTrans(const Eigen::MatrixXd &th, int c, const Eigen::VectorXd &a, const Eigen::VectorXd &d, const Eigen::VectorXd &alpha)
    {
        Eigen::Matrix4d A_1 = AH(0, th, c, a, d, alpha);
        Eigen::Matrix4d A_2 = AH(1, th, c, a, d, alpha);
        Eigen::Matrix4d A_3 = AH(2, th, c, a, d, alpha);
        Eigen::Matrix4d A_4 = AH(3, th, c, a, d, alpha);
        Eigen::Matrix4d A_5 = AH(4, th, c, a, d, alpha);
        Eigen::Matrix4d A_6 = AH(5, th, c, a, d, alpha);

        Eigen::Matrix4d T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6;

        return T_06;
    }

    Eigen::Matrix<double, dof, 1> inverseKinematics(const Eigen::Matrix<double, 6, 1> &des_coordinates, int index)
    {
        Eigen::VectorXd d = Eigen::Map<const Eigen::Matrix<double, dof, 1>>(dh_d.data());
        Eigen::VectorXd a = Eigen::Map<const Eigen::Matrix<double, dof, 1>>(dh_a.data());
        Eigen::VectorXd alpha = Eigen::Map<const Eigen::Matrix<double, dof, 1>>(dh_alpha.data());

        Eigen::MatrixXd th(6, 8);
        th.setZero();

        Eigen::MatrixXd desired_pos = Eigen::MatrixXd::Zero(4, 4);
        desired_pos.block(0, 0, 3, 3) = eulerZYZToRotM(des_coordinates.segment(3, 3));
        desired_pos.block(0, 3, 3, 1) = des_coordinates.segment(0, 3);
        desired_pos(3, 3) = 1;

        Eigen::Vector4d P_05 = desired_pos * Eigen::Vector4d(0, 0, -d(5), 1) - Eigen::Vector4d(0, 0, 0, 1);

        // **** theta1 ****
        double psi = atan2(P_05(1), P_05(0));
        double phi = acos(d(3) / std::sqrt(P_05(0) * P_05(0) + P_05(1) * P_05(1)));

        // The two solutions for theta1 correspond to the shoulder
        // being either left or right
        th.block(0, 0, 1, 4).setConstant((M_PI / 2) + psi + phi);
        th.block(0, 4, 1, 4).setConstant((M_PI / 2) + psi - phi);

        // **** theta5 ****
        Eigen::VectorXi cl(2);
        cl << 0, 4;
        for (int i = 0; i < cl.size(); ++i)
        {
            int c = cl(i);
            Eigen::Matrix4d T_10 = AH(0, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_16 = T_10 * desired_pos;
            th.block(4, c, 1, 2).setConstant(acos((T_16(2, 3) - d(3)) / d(5)));
            th.block(4, c + 2, 1, 2).setConstant(-acos((T_16(2, 3) - d(3)) / d(5)));
        }

        // **** theta6 ****
        // theta6 is not well-defined when sin(theta5) = 0
        // or when T16(1,3), T16(2,3) = 0.
        cl.resize(4);
        cl << 0, 2, 4, 6;
        for (int i = 0; i < cl.size(); ++i)
        {
            int c = cl(i);
            Eigen::Matrix4d T_10 = AH(0, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_16 = (T_10 * desired_pos).inverse();
            th.block(5, c, 1, 2).setConstant(atan2((-T_16(1, 2) / sin(th(4, c))), (T_16(0, 2) / sin(th(4, c)))));
        }

        // **** theta3 ****
        cl.resize(4);
        cl << 0, 2, 4, 6;
        for (int i = 0; i < cl.size(); ++i)
        {
            int c = cl(i);
            Eigen::Matrix4d T_10 = AH(0, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_65 = AH(5, th, c, a, d, alpha);
            Eigen::Matrix4d T_54 = AH(4, th, c, a, d, alpha);
            Eigen::Matrix4d T_14 = (T_10 * desired_pos) * (T_54 * T_65).inverse();
            Eigen::Vector4d P_13 = T_14 * Eigen::Vector4d(0, -d(3), 0, 1) - Eigen::Vector4d(0, 0, 0, 1);
            double t3 = acos((P_13.norm() * P_13.norm() - a(1) * a(1) - a(2) * a(2)) / (2 * a(1) * a(2)));
            th(2, c) = t3;
            th(2, c + 1) = -t3;
        }

        // **** theta2 and theta 4 ****
        cl.resize(8);
        cl << 0, 1, 2, 3, 4, 5, 6, 7;
        for (int i = 0; i < cl.size(); ++i)
        {
            int c = cl(i);
            Eigen::Matrix4d T_10 = AH(0, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_65 = AH(5, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_54 = AH(4, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_14 = (T_10 * desired_pos) * T_65 * T_54;
            Eigen::Vector4d P_13 = T_14 * Eigen::Vector4d(0, -d(3), 0, 1) - Eigen::Vector4d(0, 0, 0, 1);

            // theta 2
            th(1, c) = -atan2(P_13(1), -P_13(0)) + asin(a(2) * sin(th(2, c)) / P_13.norm());

            // theta 4
            Eigen::Matrix4d T_32 = AH(2, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_21 = AH(1, th, c, a, d, alpha).inverse();
            Eigen::Matrix4d T_34 = T_32 * T_21 * T_14;
            th(3, c) = atan2(T_34(1, 0), T_34(0, 0));
        }

        th = th.real();
        th = th.col(index);

        return th;
    }

    Eigen::Matrix<double, dof, dof> getJacobian(Eigen::Matrix<double, dof, 1> q)
    {
        Eigen::Matrix<double, 4, 4> T = getTransformation(0, dof, q);
        Eigen::Matrix<double, 3, 1> pe = T.block(0, 3, 3, 1);

        Eigen::Matrix<double, dof, dof> J;
        J.setZero();

        Eigen::Matrix<double, 3, 1> zi_m_1;
        zi_m_1 << 0, 0, 1;
        Eigen::Matrix<double, 3, 1> pi_m_1;
        pi_m_1 << 0, 0, 0;

        J.block(0, 0, 3, 1) = zi_m_1.cross(pe - pi_m_1);
        J.block(3, 0, 3, 1) = zi_m_1;

        for (int i = 1; i < dof; i++)
        {
            Eigen::Matrix<double, 4, 4> Ti_m_1 = getTransformation(0, i, q);

            Eigen::Matrix<double, 3, 1> zi_m_1 = Ti_m_1.block(0, 2, 3, 1);
            Eigen::Matrix<double, 3, 1> pi_m_1 = Ti_m_1.block(0, 3, 3, 1);

            J.block(0, i, 3, 1) = zi_m_1.cross(pe - pi_m_1);
            J.block(3, i, 3, 1) = zi_m_1;
        }

        return J;
    }

    Eigen::Matrix<double, dof, dof> getAnalyticalJacobian(Eigen::Matrix<double, dof, 1> q)
    {
        Eigen::Matrix<double, 4, 4> T = getTransformation(0, dof, q);
        Eigen::Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);

        double theta = atan2(sqrt(R(0, 2) * R(0, 2) + R(1, 2) * R(1, 2)), R(2, 2));
        double phi = atan2(R(1, 2), R(0, 2));
        double psi = atan2(R(2, 1), -R(2, 0));

        Eigen::Matrix<double, 3, 3> Transformation;
        Transformation << 0, -sin(phi), cos(phi) * sin(theta),
            0, cos(phi), sin(phi) * sin(theta),
            1, 0, cos(theta);

        Eigen::Matrix<double, dof, dof> Ta;
        Ta.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
        Ta.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Zero();
        Ta.block<3, 3>(3, 0) = Eigen::Matrix<double, 3, 3>::Zero();
        Ta.block<3, 3>(3, 3) = Transformation.inverse();

        return Ta * getJacobian(q);
    }
}