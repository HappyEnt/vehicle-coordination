class ParticleFilterOpenCL
{
public:
  ParticleFilterOpenCL();
  virtual ~ParticleFilterOpenCL();

  void correct();
  void predict();
  void set_particles();
  void get_particles();

private:
  void resample();
  void low_variance_resampling(struct weighted_particle *weighted_particles, struct particle* resampled_particles, size_t length);

  void normalize();
  void calculate_likelihood();
}
