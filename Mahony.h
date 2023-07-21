class Mahony
{
  public:

    Mahony();
    void upDate(float wx, float wy, float wz, float ax, float ay, float az, float mx, float my, float mz);
    float getpitch();
    float getyaw();
    float getroll();

  private:

    float intex, intey, intez;
    float pitch;
    float yaw;
    float roll;
    float q0, q1, q2, q3;

};
