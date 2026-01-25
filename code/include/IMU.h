/* ライブラリ導入 */
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用のライブラリ
#include <Wire.h>

struct IMU_Data_Vector3{
    double x;
    double y;
    double z;
    //オイラー角の場合はx=Yaw,y=Pitch,z=Rollに対応
};

struct IMU_Data_All{
    // 角速度データ、加速度データ、磁気データの全てをまとめた構造体
    IMU_Data_Vector3 gyro;
    IMU_Data_Vector3 accel;
    IMU_Data_Vector3 mag;
    IMU_Data_Vector3 euler;
};

class IMU_Manager {
    private:
        Adafruit_BNO055 bno;
        float gyro_x_offset = 0.0, gyro_y_offset = 0.0, gyro_z_offset = 0.0;
        float accel_x_offset = 0.0, accel_y_offset = 0.0, accel_z_offset = 0.0;
        float mag_x_offset = 0.0, mag_y_offset = 0.0, mag_z_offset = 0.0;
        float imu_euler_offset_yaw = 0.0, imu_euler_offset_pitch = 0.0, imu_euler_offset_roll = 0.0;
    public:
        /* BNO055用変数 */
        IMU_Manager() : bno(55, 0x29, &Wire1) {}
        //初期化
        bool begin(){
            Wire1.begin();
            if(!bno.begin()){
                return false;
            }
            delay(50);
            waitforCalibration();
            IMU_Reset();
            return true;
        }
    void waitforCalibration(){
        uint8_t system, gyro, accel, mag;
        uint32_t start = millis();
        const uint32_t timeout = 3000;
        do{
            bno.getCalibration(&system, &gyro, &accel, &mag);
            delay(20);
        }while(system < 3 && millis() - start < timeout);
    }
    void IMU_Reset(){
        const int N = 5;

        float gx=0.0, gy=0.0, gz=0.0;
        float ax=0.0, ay=0.0, az=0.0;
        float mx=0.0, my=0.0, mz=0.0;
        float yaw=0.0, pitch=0.0, roll=0.0;

        for(int i=0; i<N; i++){
            imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

            gx += gyroscope.x();
            gy += gyroscope.y();
            gz += gyroscope.z();

            ax += accelermetor.x();
            ay += accelermetor.y();
            az += accelermetor.z();

            mx += magnetometer.x();
            my += magnetometer.y();
            mz += magnetometer.z();

            yaw += euler.x();
            pitch += euler.y();
            roll += euler.z();

            delay(5);
        }

        gyro_x_offset = gx / N;
        gyro_y_offset = gy / N;
        gyro_z_offset = gz / N;

        accel_x_offset = ax / N;
        accel_y_offset = ay / N;
        accel_z_offset = az / N;

        mag_x_offset = mx / N;
        mag_y_offset = my / N;
        mag_z_offset = mz / N;

        imu_euler_offset_yaw = yaw / N;
        imu_euler_offset_pitch = pitch / N;
        imu_euler_offset_roll = roll / N;
    }
    //各種データ取得関数
    // 角速度データ取得 単位は[rad/s]
    IMU_Data_Vector3 Get_IMU_GYRO(){
        imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        return IMU_Data_Vector3{gyroscope.x() - gyro_x_offset, gyroscope.y() - gyro_y_offset, gyroscope.z() - gyro_z_offset};
    }

    // 加速度データ取得 単位は[m/s^2]
    IMU_Data_Vector3 Get_IMU_ACCEL(){
        imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        return IMU_Data_Vector3{accelermetor.x() - accel_x_offset, accelermetor.y() - accel_y_offset, accelermetor.z() - accel_z_offset};
    }

    // 磁気データ取得 単位は[μT]
    IMU_Data_Vector3 Get_IMU_MAG(){
        imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        return IMU_Data_Vector3{magnetometer.x() - mag_x_offset, magnetometer.y() - mag_y_offset, magnetometer.z() - mag_z_offset};
    }

    // オイラー角データ取得 単位は[degree]
    //xはYaw,yはPitch,zはRollに対応
    IMU_Data_Vector3 Get_IMU_EULER(){
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        //0°～360°の範囲に補正
        float yaw = euler.x() - imu_euler_offset_yaw;
        float pitch = euler.y() - imu_euler_offset_pitch;
        float roll = euler.z() - imu_euler_offset_roll;
        if(yaw < 0) yaw += 360.0;
        else if(yaw >= 360.0) yaw -= 360.0;
        if(pitch < 0) pitch += 360.0;
        else if(pitch >= 360.0) pitch -= 360.0;
        if(roll < 0) roll += 360.0;
        else if(roll >= 360.0) roll -= 360.0;
        
        //-180°～180°の範囲に変換
        if(yaw > 180.0) yaw -= 360.0;
        if(pitch > 180.0) pitch -= 360.0;
        if(roll > 180.0) roll -= 360.0;

        //最終チェック
        if(yaw < -180.0) yaw = -180.0;
        else if(yaw > 180.0) yaw = 180.0;
        if(pitch < -180.0) pitch = -180.0;
        else if(pitch > 180.0) pitch = 180.0;
        if(roll < -180.0) roll = -180.0;
        else if(roll > 180.0) roll = 180.0;
        return IMU_Data_Vector3{yaw, pitch, roll};
    }

    IMU_Data_All Get_IMU_All(){
        return {
            Get_IMU_GYRO(),
            Get_IMU_ACCEL(),
            Get_IMU_MAG(),
            Get_IMU_EULER()
        };
    }
};
