struct _PID
{
  float KP;
  float KI;
  float KD;

  float expectation;
  float measurement;

  float integral;
  float intmax;
  float intmin;

  float err;
  float lasterr;
  float out;

};

void PID_init(struct _PID* p, float kp, float ki, float kd)
{
  p->expectation = 1.0;
  p->measurement = 0.0;

  p->integral = 0.0;
  p->intmax = 5.0;
  p->intmin = -5.0;

  p->err = 0.0;
  p->lasterr = 0.0;
  p->out = 0.0;

  p->KP = kp;
  p->KI = ki;
  p->KD = kd;
}

float PID_update(struct _PID* p, float measurement, float expectation)
{
  int index;
  p->expectation = expectation;
  p->measurement = measurement;
  p->err = p->expectation - p->measurement;

  if (p->integral > p-> intmax)
  {
    if (abs(p->err) > 200)
    {
      index = 0;
    }
    else {
      index = 1;
      if (p->err < 0)
      {
        p->integral += p->err;
      }
    }
  }
  if (p->integral < p->intmin)
  {
    if (abs(p->err) > 200)
    {
      index = 0;
    }
    else
    {
      index = 1;
      if (p->err > 0)
      {
        p->integral += p->err;
      }
    }

  }
  else
  {
    if (abs(p->err) > 200)
    {
      index = 0;
    }
    else
    {
      index = 1;
      p->integral += p->err;
    }
  }
  p->out = p->KP * p->err + index * p->KI * p->integral + p->KD * (p->err - p->lasterr);
  p->lasterr = p->err;
  return p->out;
}
