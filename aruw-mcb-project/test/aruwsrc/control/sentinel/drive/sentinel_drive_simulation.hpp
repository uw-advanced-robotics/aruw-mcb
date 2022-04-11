class SentinelDriveSimulation
{
    public:
        void setDesiredRpm(float desRpm);

        float getRpm();

        float absolutePosition();

    private:
        float currentRpm = 0;
        float currentPosition;
};

