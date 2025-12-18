/* ライブラリ導入 */
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用のライブラリ
#include <Wire.h>

struct IMU_Data_Vector3{
    double x;
    double y;
    double z;
};

struct IMU_Data_All{
    // 角速度データ、加速度データ、磁気データの全て 
    IMU_Data_Vector3 gyro;
    IMU_Data_Vector3 accel;
    IMU_Data_Vector3 mag;
    IMU_Data_Vector3 euler;
};

class IMU_Manager {
    private:
        Adafruit_BNO055 bno;
    public:
        /* BNO055用変数 */
        IMU_Manager() : bno(55, 0x29, &Wire1) {}
        //初期化
        bool begin(){
            Wire1.begin();
            if(!bno.begin()){
                return false;
            }
            return true;
        }
    //各種データ取得関数
    // 角速度データ取得 単位は[rad/s]
    IMU_Data_Vector3 Get_IMU_GYRO(){
        imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        return IMU_Data_Vector3{gyroscope.x(), gyroscope.y(), gyroscope.z()};
    }

    // 加速度データ取得 単位は[m/s^2]
    IMU_Data_Vector3 Get_IMU_ACCEL(){
        imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        return IMU_Data_Vector3{accelermetor.x(), accelermetor.y(), accelermetor.z()};
    }

    // 磁気データ取得 単位は[μT]
    IMU_Data_Vector3 Get_IMU_MAG(){
        imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        return IMU_Data_Vector3{magnetometer.x(), magnetometer.y(), magnetometer.z()};
    }

    // オイラー角データ取得 単位は[degree]
    IMU_Data_Vector3 Get_IMU_EULER(){
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        return IMU_Data_Vector3{euler.x(), euler.y(), euler.z()};
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
