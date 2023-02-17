#include <memory>

#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(penetration_allowance, 1.0E-3, "Allowable penetration [m]");
DEFINE_double(time_step, 1.0E-3, "Simulation time step [sec]");
DEFINE_double(simulation_time, 10.0, "Simulation time [sec]");
DEFINE_double(mbp_discrete_update_period, 0.01, "The fixed-time step period (in seconds) of discrete updates");

namespace drake {
namespace examples {
namespace cheetah {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using Eigen::Translation3d;
using Eigen::VectorXd;

int do_main() {
    systems::DiagramBuilder<double> builder;

    MultibodyPlantConfig plant_config;
    plant_config.time_step = FLAGS_time_step;
    plant_config.penetration_allowance = FLAGS_penetration_allowance;

    auto [plant, scene_graph] = multibody::AddMultibodyPlant(plant_config, &builder);

    multibody::Parser(&plant).AddModelFromFile("/home/nathant/drake/examples/quadruped/models/mini_cheetah/mini_cheetah_mesh.urdf");

    const double static_friction = 1.0;
    const double dynamic_friction = static_friction;
    const multibody::CoulombFriction<double> ground_friction (static_friction, dynamic_friction);
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(), geometry::HalfSpace(), "GroundCollisionGeometry", ground_friction);

    plant.Finalize();

    DRAKE_DEMAND(plant.num_velocities() == 18);
    DRAKE_DEMAND(plant.num_positions() == 19);

    visualization::AddDefaultVisualization(&builder);

    auto diagram = builder.Build();

    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    systems::Context<double>& plant_context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    const VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
    plant.get_actuation_input_port().FixValue(&plant_context, tau);

    const multibody::Body<double>& body = plant.GetBodyByName("body");
    DRAKE_DEMAND(body.is_floating());

    const Translation3d X_WB (0.0, 0.0, 0.3);
    plant.SetFreeBodyPoseInWorldFrame(&plant_context, body, X_WB);

    auto simulator = MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
    simulator->AdvanceTo(FLAGS_simulation_time);

    return 0;
}
     
} // namespace
} // namespace cheetah
} // namespace examples
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage("Passive simulation of Mini Cheetah\n");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    
    return drake::examples::cheetah::do_main();
}