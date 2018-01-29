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

void TxEncodeVehicleData(int8_t stateMode,float x,float y,float heading,float yawAngle,float yawRt,float vel,float odo);
void TxEncodeCourseData(int CourseID,float xNext,float yNext,float headNext,float phiV,float phiU,float h);
void AddCheckSum(uint8_t *arryToAddCheckSum,uint8_t nByte);

void TxEncodeVehicleData(int8_t stateMode,float x,float y,float heading,float yawAngle,float yawRt,float vel,float odo)
{
    union sI32ToByte I32Bx,I32By;
    union sI16ToByte I16Bheading,I16ByawAngle,I16ByawRt;
    union uI16ToByte uI16Bvel,uI16Bodo;

    uint8_t header[2] = {0xEC,0xAB};
    uint8_t footer[2] = {0xED,0xDA};
    uint8_t vehicleDatabuf[32] = {};
    uint8_t dID = 0x01;
    //車両データ 整数配列化して格納
    memcpy(&vehicleDatabuf[0],&dID,1);
    memcpy(&vehicleDatabuf[1],&stateMode,1);

    I32Bx.integer = (int32_t)(x * 100);memcpy(&vehicleDatabuf[2],I32Bx.byte,4);
    I32By.integer = (int32_t)(y * 100);memcpy(&vehicleDatabuf[6],I32By.byte,4);
    I16Bheading.integer = (int16_t)(heading * 10000);memcpy(&vehicleDatabuf[10],I16Bheading.byte,2);
    I16ByawAngle.integer = (int16_t)(yawAngle * 10000);memcpy(&vehicleDatabuf[12],I16ByawAngle.byte,2);
    I16ByawRt.integer = (int16_t)(yawRt * 1000);memcpy(&vehicleDatabuf[14],I16ByawRt.byte,2);
    uI16Bvel.integer = (uint16_t)(vel * 100);memcpy(&vehicleDatabuf[16],uI16Bvel.byte,2);
    uI16Bodo.integer = (uint16_t)(odo * 100);memcpy(&vehicleDatabuf[18],uI16Bodo.byte,2);

    AddCheckSum(vehicleDatabuf,sizeof(vehicleDatabuf));
    Serial.write(header,sizeof(header));
    Serial.write(vehicleDatabuf,sizeof(vehicleDatabuf));
    Serial.write(footer,sizeof(footer));
}

void TxEncodeCourseData(int CourseID,float xNext,float yNext,float headNext,float phiV,float phiU,float h)
{
    static int lastCourseID = 0;
//コースID変更時のみ送信
if(CourseID != lastCourseID){
        union sI32ToByte I32BxNext,I32ByNext;
        union sI16ToByte I16BheadNext,I16BphiV,I16BphiU;
        union uI16ToByte uI16Bh;

        uint8_t header[2] = {0xEC,0xAB};
        uint8_t footer[2] = {0xED,0xDA};
        uint8_t courseDatabuf[32] = {};
        uint8_t dID = 0x02;

        //コースデータ 整数配列化して格納
        memcpy(&courseDatabuf[0],&dID,1);
        memcpy(&courseDatabuf[2],&CourseID,2);
        I32ByNext.integer = (int32_t)(xNext * 100);memcpy(&courseDatabuf[4],I32BxNext.byte,4);
        I32ByNext.integer = (int32_t)(yNext * 100);memcpy(&courseDatabuf[8],I32ByNext.byte,4);
        I16BheadNext.integer = (int16_t)(headNext * 10000);memcpy(&courseDatabuf[12],I16BheadNext.byte,2);
        I16BphiV.integer = (int16_t)(phiV * 1000);memcpy(&courseDatabuf[14],I16BphiV.byte,2);
        I16BphiU.integer = (int16_t)(phiU * 1000);memcpy(&courseDatabuf[16],I16BphiU.byte,2);
        uI16Bh.integer = (uint16_t)(h * 100);memcpy(&courseDatabuf[18],uI16Bh.byte,2);

        AddCheckSum(courseDatabuf,sizeof(courseDatabuf));
        Serial.write(header,sizeof(header));
        Serial.write(courseDatabuf,sizeof(courseDatabuf));
        Serial.write(footer,sizeof(footer));
    }

    lastCourseID = CourseID;
}

void AddCheckSum(uint8_t *arryToAddCheckSum,uint8_t nByte)
{
    uint8_t checkSum = 0;
    for(uint8_t i=0;i<(nByte-1);i++){
        checkSum += *(arryToAddCheckSum + i);
    }
    *(arryToAddCheckSum + nByte-1) = checkSum;
}