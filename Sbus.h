class Sbus
{
public:
    Sbus();
    void SbusRead(void);
    unsigned int* GetCh();
    unsigned int* GetOffset();

private:
    bool sbusFlag = false;
    unsigned int sbusBuffer[18];
    unsigned int ch[12];
    unsigned int offset[8];
};