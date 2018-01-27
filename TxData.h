union sI32ToByte
{
    int32_t integer;
    uint8_t byte[sizeof(integer)];
};

union sI16ToByte
{
    int16_t integer;
    uint8_t byte[sizeof(integer)];
};

union uI16ToByte
{
    uint16_t integer;
    uint8_t byte[sizeof(integer)];
};

void TxEncodeVehicleData(float x,float y,float heading,float yawRt,float vel);
void TxEncodeCourseData(float phiV,float phiU,float h,float odo);
void AddCheckSum(uint8_t *arryToAddCheckSum,uint8_t nByte);

void TxEncodeVehicleData(float x,float y,float heading,float yawRt,float vel)
{
    union sI32ToByte I32Bx,I32By;
    union sI16ToByte I16Bheading,I16ByawRt;
    union uI16ToByte uI16Bvel;

    uint8_t vehicleDatabuf[32] = {};

    //車両データ 整数配列化して格納
    I32Bx.integer = (int32_t)(x * 100);memcpy(&vehicleDatabuf[0],I32Bx.byte,4);
    I32By.integer = (int32_t)(y * 100);memcpy(&vehicleDatabuf[4],I32By.byte,4);
    I16Bheading.integer = (int16_t)(heading * 10000);memcpy(&vehicleDatabuf[8],I16Bheading.byte,2);
    I16ByawRt.integer = (int16_t)(yawRt * 1000);memcpy(&vehicleDatabuf[10],I16ByawRt.byte,2);
    uI16Bvel.integer = (uint16_t)(vel * 100);memcpy(&vehicleDatabuf[12],uI16Bvel.byte,2);

    AddCheckSum(vehicleDatabuf,sizeof(vehicleDatabuf));
    Serial.write(vehicleDatabuf,sizeof(vehicleDatabuf));
}

void TxEncodeCourseData(float phiV,float phiU,float h,float odo)
{
    union sI16ToByte I16BphiV,I16BphiU;
    union uI16ToByte uI16Bh,uI16Bodo;

    uint8_t courseDatabuf[16] = {};
    
    //コースデータ 整数配列化して格納
    I16BphiV.integer = (int16_t)(phiV * 1000);memcpy(&courseDatabuf[0],I16BphiV.byte,2);
    I16BphiU.integer = (int16_t)(phiU * 1000);memcpy(&courseDatabuf[2],I16BphiU.byte,2);
    uI16Bh.integer = (uint16_t)(h * 100);memcpy(&courseDatabuf[4],uI16Bh.byte,2);
    uI16Bodo.integer = (uint16_t)(odo * 100);memcpy(&courseDatabuf[6],uI16Bodo.byte,2);

    AddCheckSum(courseDatabuf,sizeof(courseDatabuf));
    Serial.write(courseDatabuf,sizeof(courseDatabuf));
}

void AddCheckSum(uint8_t *arryToAddCheckSum,uint8_t nByte)
{
    uint8_t checkSum = 0;
    for(uint8_t i=0;i<(nByte-1);i++){
        checkSum += *(arryToAddCheckSum + i);
    }
    *(arryToAddCheckSum + nByte-1) = checkSum;
}