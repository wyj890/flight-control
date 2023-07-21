class lowpassfilter
{
  public:
    lowpassfilter(float sample_rate, float cutoff_frequency)
    {
      float dt = 1.0f / sample_rate;
      float RC = 1.0f / (cutoff_frequency * 2.0 * M_PI);
      alpha_ = dt / (dt + RC);
      prev_output_ = 0.0f;
    }

    float Update(float input)
    {
      float output = alpha_ * input + (1.0 - alpha_) * prev_output_;
      prev_output_ = output;
      return output;
    }

  private:
    float alpha_;
    float prev_output_;
};
