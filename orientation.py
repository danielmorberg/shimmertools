import math

class Orientation:
    q1 = q2 = q3 = q4 = mBeta = samplingperiod = 0.0

    def __init__(self, mBeta, samplingperiod, q1, q2, q3, q4):
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        self.mBeta = mBeta
        self.mSamplingPeriod = samplingperiod

    def quat_update(self,ax, ay, az, gx, gy, gz, mx, my, mz):
        # Just a python port of the code Shimmer uses in their GradDes3DOrientation.cs file in the Shimmer-C-API project:
        # https://github.com/ShimmerEngineering/Shimmer-C-API/blob/master/ShimmerAPI/ShimmerAPI/GradDes3DOrientation.cs

        # Their reference:
        # Madgwick, Sebastian OH, Andrew JL Harrison, "Estimation of imu and marg orientation using a gradient descent algorithm." Rehabilitation Robotics (ICORR), 2011 IEEE International Conference on. IEEE, 2011.
        # Their 3D orientation code is taken from https://code.google.com/p/labview-quaternion-ahrs/ which is licensed under GNU_Lesser_GPL

        norm = hx = hy = _2bx = _2bz = s1 = s2 = s3 = s4 = None
        qDot1 = qDot2 = qDot3 = qDot4 = None
        _4bx = _4bz = _2q1 = _2q2 = _2q3 = _2q4 = None
        q1q1 = q1q2 = q1q3 = q1q4 = q2q2 = q2q3 = q2q4 = q3q3 = q3q4 = q4q4 = _2q1q3 = _2q3q4 = None


        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        q4 = self.q4
        #norm = self.norm
        mBeta = self.mBeta
        mSamplingPeriod = self.mSamplingPeriod

        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        norm = math.sqrt(ax * ax + ay * ay + az * az);
        if norm > 0.0:
            norm = 1.0 / norm
            ax *= norm
            ay *= norm
            az *= norm

        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm > 0.0:
            norm = 1.0 / norm
            mx *= norm
            my *= norm
            mz *= norm

        hx = mx * q1q1 - (2 * q1 * my) * q4 + (2 * q1 * mz) * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = (2 * q1 * mx) * q4 + my * q1q1 - (2 * q1 * mz) * q2 + (2 * q2 * mx) * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -(2 * q1 * mx) * q3 + (2 * q1 * my) * q2 + mz * q1q1 + (2 * q2 * mx) * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        norm = 1.0 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - mBeta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - mBeta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - mBeta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - mBeta * s4

        q1 += qDot1 * mSamplingPeriod
        q2 += qDot2 * mSamplingPeriod
        q3 += qDot3 * mSamplingPeriod
        q4 += qDot4 * mSamplingPeriod
        norm = 1.0 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)

        q1 = q1 * norm
        q2 = q2 * norm
        q3 = q3 * norm
        q4 = q4 * norm
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        #self.norm = norm


        return[q1, q2, q3, q4]
