#include <gazebo/gazebo.hh>
class PIDController {
    public:
        PIDController();
        virtual ~PIDController();
        virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");
        double update(double input, double x, double dx, double dt);
        void reset();
    private:
        double gain_p;
        double gain_i;
        double gain_d;
        double time_constant;
        double limit;

        double input;
        double dinput;
        double output;
        double p, i, d;
};