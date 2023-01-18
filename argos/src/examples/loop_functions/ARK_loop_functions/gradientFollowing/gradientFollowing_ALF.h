/**
 * @file <gradientFollowing_ALF.h>
 *
 * @author Luigi Feola <feola@diag.uniroma1.it>
 *
 * @brief This is the source file of the ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 */

#ifndef GRADIENTFOLLOWING_ALF_H
#define GRADIENTFOLLOWING_ALF_H

namespace argos
{
    class CSpace;
    class CFloorEntity;
    class CSimulator;
}

typedef enum
{
    kBLACK = 0,
    kWHITE = 2,
    kGRAY = 1,
    kLIGHTGRAY = 3,
} light_sensor;


#include <math.h>
#include <bitset>
#include <numeric>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

// kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>
#include <random>
#include <algorithm>
using namespace argos;

class GradientFollowingCALF : public CALF
{

public:
    GradientFollowingCALF();

    virtual ~GradientFollowingCALF() {}

    virtual void Init(TConfigurationNode &t_tree);

    virtual void PlaceQuarterCircleWall(CVector3 pos, double radius, unsigned int walls, unsigned int quadrant);

    virtual void PlaceBots(CVector3 arenaSize, double cornerRadius);

    virtual void Reset();

    virtual void Destroy();

    virtual void PostStep();

    virtual void PostExperiment();

    /** Get a Vector of all the Kilobots in the space */
    void GetKilobotsEntities();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode &t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode &t_tree);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Get the sensor reading to send to a Kilobot according to its position */
    Real addNoise(Real value);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2 &vec_position_on_plane);

    /** 2D vector rotation */
    CVector2 VectorRotation2D(Real angle, CVector2 vec);

    /** Simulate proximity sensor*/
    std::vector<int> Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors);

    /** Log Kilobot pose and state */
    void KiloLOG();

    

private:
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/

    /** virtual environment struct*/
    struct Gradient
    {
        CVector2 CenterPos;
    };

    /***********************************/
    /*      Experiment variables       */
    /***********************************/
    /* random number generator */
    CRandom::CRNG *c_rng;
    UInt32 random_seed;

    /* Send a right amount of messages for each time-step*/
    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /* output LOG files */
    std::ofstream m_kiloOutput;
    std::string m_strKiloOutputFileName;

    /** Size of social robots */
    unsigned int socialRobots;

    /* Kilobots properties */
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<CRadians> m_vecKilobotsOrientations;
    std::vector<Real> m_vecKilobotsLightSensors;

    /** Gradient field radius */
    Real m_fGradientFieldRadius;

    /** Circular corner radius  */
    double cornerRadius;

    /** output file for data acquisition */
    std::ofstream m_cOutput;

    /** output file name*/
    std::string m_strOutputFileName;

    /** data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;

    std::default_random_engine generator;
    std::normal_distribution<Real> distribution;

};

#endif
