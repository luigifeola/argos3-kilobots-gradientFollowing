/**
 * @file <gradientFollowing_ALF.cpp>
 *
 * @author Luigi Feola <feola@diag.uniroma1.it>
 *
 * @brief This is the source file of the ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 */

#include "gradientFollowing_ALF.h"

namespace
{

    const double kKiloDiameter = 0.033;
    double vArena_size = 1.0;
    double vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;

    const UInt32 MAX_PLACE_TRIALS = 100;

    // wall avoidance params
    const CVector2 up_direction(0.0, -1.0);
    const CVector2 down_direction(0.0, 1.0);
    const CVector2 left_direction(1.0, 0.0);
    const CVector2 right_direction(-1.0, 0.0);
    const int kProximity_bits = 8;

    // log time counter
    int internal_counter = 0;

    typedef enum
    {
        CONTINUOUS = 0,
        DISCRETE = 1,
        NO_BACKGROUND = 2
    } Background;

    typedef enum
    {
        BITS_2 = 2,
        BITS_3 = 3,
        BITS_4 = 4
    } Discretization;

    Background background_flag = DISCRETE;
    Discretization discret_bits = BITS_3;

    const Real MAX_VAL = 1.0;
    const Real NUM_SYMBOLS = Real(discret_bits);
    Real overall_gradient = 0.0;
    Real gradient_radius = 0.5;
}

/****************************************/
/****************************************/

GradientFollowingCALF::GradientFollowingCALF() : m_unDataAcquisitionFrequency(10),
                                                 generator(), distribution(0.0, 0.1)
{
    c_rng = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void GradientFollowingCALF::Init(TConfigurationNode &t_node)
{
    std::cout<< "Init\n";

    /* Initialize ALF*/
    CALF::Init(t_node);
    random_seed = GetSimulator().GetRandomSeed();

    /*********** LOG FILES *********/
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);


    
}

/****************************************/
/****************************************/

void GradientFollowingCALF::Reset()
{
    m_kiloOutput.close();
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);

}

/****************************************/
/****************************************/

void GradientFollowingCALF::Destroy()
{
    m_kiloOutput.close();
}

/****************************************/
/****************************************/

void GradientFollowingCALF::PostStep()
{
    internal_counter += 1;
    if (internal_counter % m_unDataAcquisitionFrequency == 0 || internal_counter <= 1)
    {
        KiloLOG();
    }

    // std::cout << "Overall gradient:" << (overall_gradient / m_tKilobotEntities.size()) / internal_counter << std::endl;
}

/****************************************/
/****************************************/

void GradientFollowingCALF::SetupInitialKilobotStates()
{
    std::cout<< "SetupInitialKilobotStates\n";
    /* Resize variables related to the number of Kilobots */
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsLightSensors.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    if(socialRobots > m_tKilobotEntities.size())
    {
        std::cerr << "Asked to many social robots\n";
        exit(-1);
    }

    /* Setup the min time between two message sent to a kilobot (ARK message sending limits)*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        /* Setup the virtual states of a kilobot*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    CVector3 arena_size = GetSpace().GetArenaSize();
    PlaceBots(arena_size, cornerRadius);
}

/****************************************/
/****************************************/

void GradientFollowingCALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    /* Get the robot ID */
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    
    
    if(unKilobotID < socialRobots)
        c_kilobot_entity.GetControllableEntity().SetController("social_behavior");
}

/****************************************/
/****************************************/


void GradientFollowingCALF::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    std::cout<< "SetupVirtualEnvironments\n";
    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    std::cout << "Arena size: " << vArena_size << "\tWA threshold: " << vDistance_threshold << "\n";

    /* Get the experiment variables node from the .argos file */
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");

    double cornerProportion;
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "cornerProportion", cornerProportion, 0.1);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "socialRobots", socialRobots, socialRobots);
    // std::cout << std::endl << "cornerProportion: " << cornerProportion << std::endl << std::endl;
    std::cout << std::endl << "socialRobots: " << socialRobots << std::endl << std::endl;

    /**
     * 
     *  CREATION AND POSITIONING OF THE ARENA WALLS
     *  
     * */
    CVector3 arena_size = GetSpace().GetArenaSize();

    unsigned int cornerWalls = 20;
    cornerRadius = cornerProportion * Min(arena_size[0],arena_size[1]);

    CQuaternion wall_orientation;
    wall_orientation.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO );
    CBoxEntity* box = new CBoxEntity("west_wall", CVector3(0,-arena_size[1]/2,0), wall_orientation, false, CVector3(arena_size[0]-2*cornerRadius,0.01,0.01), (Real)1.0 );
    AddEntity( *box );
    box = new CBoxEntity("east_wall", CVector3(0,arena_size[1]/2,0), wall_orientation, false, CVector3(arena_size[0]-2*cornerRadius,0.01,0.01), (Real)1.0 );
    AddEntity( *box );
    box = new CBoxEntity("south_wall", CVector3(-arena_size[0]/2,0,0), wall_orientation, false, CVector3(0.01,arena_size[1]-2*cornerRadius,0.01), (Real)1.0 );
    AddEntity( *box );
    box = new CBoxEntity("north_wall", CVector3(arena_size[0]/2,0,0), wall_orientation, false, CVector3(0.01,arena_size[1]-2*cornerRadius,0.01), (Real)1.0 );
    AddEntity( *box );

    PlaceQuarterCircleWall(CVector3(arena_size[0]/2-cornerRadius,arena_size[1]/2-cornerRadius,0),cornerRadius,cornerWalls,0);
    PlaceQuarterCircleWall(CVector3(-arena_size[0]/2+cornerRadius,arena_size[1]/2-cornerRadius,0),cornerRadius,cornerWalls,1);
    PlaceQuarterCircleWall(CVector3(-arena_size[0]/2+cornerRadius,-arena_size[1]/2+cornerRadius,0),cornerRadius,cornerWalls,2);
    PlaceQuarterCircleWall(CVector3(arena_size[0]/2-cornerRadius,-arena_size[1]/2+cornerRadius,0),cornerRadius,cornerWalls,3);
}

/****************************************/
/****************************************/

void GradientFollowingCALF::PlaceQuarterCircleWall(CVector3 pos, double radius, unsigned int nbWalls, unsigned int quadrant) {
    CRadians wall_angle = CRadians::TWO_PI/(4*nbWalls);CVector3 wall_size(0.01, 2.0*radius*Tan(CRadians::PI/(4*nbWalls)), 0.01);
    std::ostringstream entity_id;
    for( UInt32 i = quadrant*nbWalls; i <= quadrant*nbWalls + nbWalls; i++ ) {
        entity_id.str("");entity_id << "wall_" <<quadrant<< i;
        CRadians wall_rotation = wall_angle*i;
        
        CVector3 wall_position(pos[0] + radius*Cos(wall_rotation), pos[1] + radius*Sin(wall_rotation), 0 );
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles( wall_rotation, CRadians::ZERO, CRadians::ZERO );
        
        CBoxEntity* box = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size, (Real)1.0 );
        AddEntity( *box );
    }
}

/****************************************/
/****************************************/

void GradientFollowingCALF::PlaceBots(CVector3 arenaSize, double cornerRadius) {
    CVector3 cPosition;
    CQuaternion cOrientation;
    cPosition.SetZ(0.0);
    CRandom::CRNG* m_pcRNG = CRandom::CreateRNG("argos");
    unsigned int unTrials;
    CKilobotEntity* pcKB;
    for(unsigned int i=0; i<m_tKilobotEntities.size(); ++i) {
        bool bDone = false;
        unTrials = 0;
        
        pcKB = m_tKilobotEntities[i];
        do {
            double x = m_pcRNG->Uniform(CRange<Real>(-1.0 * vDistance_threshold, vDistance_threshold));
            double y = m_pcRNG->Uniform(CRange<Real>(-1.0 * vDistance_threshold, vDistance_threshold));
            if(abs(x)<arenaSize[0]/2.0-cornerRadius or abs(y)<arenaSize[1]/2.0-cornerRadius or Distance(CVector3(abs(x),abs(y),0),CVector3(arenaSize[0]/2.0-cornerRadius,arenaSize[1]/2.0-cornerRadius,0))<cornerRadius) {
                cPosition.SetX(x);
                cPosition.SetY(y);

                CRadians cRandomOrientation = CRadians(m_pcRNG->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
                cOrientation.FromEulerAngles(cRandomOrientation, CRadians::ZERO, CRadians::ZERO);

                bDone = MoveEntity(pcKB->GetEmbodiedEntity(), cPosition, cOrientation);
            }
            ++unTrials;
        } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
        if(!bDone) {
            THROW_ARGOSEXCEPTION("Can't place " << "kb_" + ToString(i));
        }
    }
    
}

/****************************************/
/****************************************/

void GradientFollowingCALF::GetExperimentVariables(TConfigurationNode &t_tree)
{
    std::cout<< "GetExperimentVariables\n";
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");

    // /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "kilo_filename", m_strKiloOutputFileName);
    // std::cout<< "Filename: " << m_strKiloOutputFileName << std::endl;

    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

/****************************************/
/****************************************/

CVector2 GradientFollowingCALF::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}

/****************************************/
/****************************************/

std::vector<int> GradientFollowingCALF::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_start = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_end = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_start) >= 0.0 || obstacle_direction.DotProduct(sector_dir_end) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
}

/****************************************/
/****************************************/

void GradientFollowingCALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    // std::cout << "unKilobotID: "  << unKilobotID << " controller:" 
    //           << (unKilobotID < socialRobots ? "soc" : "env") << std::endl;
    // std::cout << "unKilobotID: "  << unKilobotID << std::endl;
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

/****************************************/
/****************************************/

Real GradientFollowingCALF::addNoise(Real value)
{
        float noise = distribution(generator);
        value += noise;
        return (value > 0.0f) ? ( (value < 1.0f) ? value : 1.0f) : 0.0f;
}

/****************************************/
/****************************************/

void GradientFollowingCALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;

    /* Flag for existence of message to send */
    bool bMessageToSend = true;

    /* Get the kilobot ID and state (Position and Orientation in this example*/
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    tKilobotMessage.m_sID = unKilobotID;
    tKilobotMessage.m_sData = 0;

    // std::cout << "unKilobotID " << unKilobotID << " c_kilobot_entity.GetId(): " << c_kilobot_entity.GetId() << std::endl;
    // std::cout<< "Tograyscale " << m_vecKilobotsLightSensors[unKilobotID].ToGrayScale() << std::endl;

    Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], CVector2(0.0, 0.0));
    Real headindIndex = fDistance/(gradient_radius);

    // Real headindIndex1 = addNoise(headindIndex);
    // Real headindIndex1 = headindIndex;
    // Real symbol1 = static_cast<int>(((headindIndex1 * NUM_SYMBOLS) / MAX_VAL)); // 0, 1, 2, 3

    Real symbol = static_cast<int>(((headindIndex * NUM_SYMBOLS) / MAX_VAL)); // 0, 1, 2, 3

    // std::cout << unKilobotID << " symbol " << symbol << std::endl;
    // std::cout << unKilobotID << " gradient " << headindIndex << " symbol " << symbol << std::endl;
    // std::cout << unKilobotID << " noise gradient " << headindIndex1 << " symbol1 " << symbol1 << std::endl << std::endl;

    m_vecKilobotsLightSensors[unKilobotID] = headindIndex;
    overall_gradient += headindIndex;

    if (symbol == 0.0)
    {
        tKilobotMessage.m_sType = kBLACK;
    }
    else if (symbol == 1.0 && Real(discret_bits) > 2)
    {
        tKilobotMessage.m_sType = kGRAY;
    }
    else if (symbol == 2.0 && Real(discret_bits) > 3)
    {
        tKilobotMessage.m_sType = kLIGHTGRAY;
    }
    else
    {
        tKilobotMessage.m_sType = kWHITE;
    }
    
    // std::cout << unKilobotID << " sending " << tKilobotMessage.m_sType << std::endl << std::endl;
    /* check for robot collisions with walls */
    UInt8 proximity_sensor_dec = 0; // 8 bit proximity sensor as decimal

    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        return;
    }

    if (fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold ||
        fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold)
    {
        // if(unKilobotID == 6)
        //     std::cout<< "kID:" << unKilobotID << "\n";

        std::vector<int> proximity_vec;
        if (m_vecKilobotsPositions[unKilobotID].GetX() > vDistance_threshold)
        {
            // if (unKilobotID == 6)
            //     std::cout << "---RIGHT\n";
            proximity_vec = Proximity_sensor(right_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
        }
        else if (m_vecKilobotsPositions[unKilobotID].GetX() < -1.0 * vDistance_threshold)
        {
            // if (unKilobotID == 6)
            //     std::cout << "---LEFT\n";
            proximity_vec = Proximity_sensor(left_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
        }

        else
        {
            if (m_vecKilobotsPositions[unKilobotID].GetY() > vDistance_threshold)
            {
                // if (unKilobotID == 6)
                //     std::cout << "---UP\n";
                proximity_vec = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
            }
            else if (m_vecKilobotsPositions[unKilobotID].GetY() < -1.0 * vDistance_threshold)
            {
                // if (unKilobotID == 6)
                //     std::cout << "---DOWN\n";
                proximity_vec = Proximity_sensor(down_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
            }
        }
        // if (unKilobotID == 6)
        // {
        //     std::cout << "\t";
        //     for(auto bit : proximity_vec){
        //         std::cout << bit;
        //     }
        //     std::cout << std::endl;
        // }
        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y)
                                                   { return (x << 1) + y; });
        /* To turn off the wall avoidance decomment the following line */
        // proximity_sensor_dec = 0;

        tKilobotMessage.m_sData = proximity_sensor_dec;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if (bMessageToSend)
    {
        // std::cout << "Sending message to kID " << unKilobotID << "\n";
        // std::cout << "m_sID " << tKilobotMessage.m_sID << "\n";
        // std::cout << "m_sType " << tKilobotMessage.m_sType << "\n";
        // std::cout << "m_sData " << tKilobotMessage.m_sData << "\n";

        for (int i = 0; i < 9; ++i)
        {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        // Prepare an empty ARK-type message to fill the gap in the full kilobot message
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;
        // Fill the kilobot message by the ARK-type messages
        for (int i = 0; i < 3; ++i)
        {
            if (i == 0)
            {
                tMessage = tKilobotMessage;
            }
            else
            {
                tMessage = tEmptyMessage;
            }
            m_tMessages[unKilobotID].data[i * 3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2 + i * 3] = tMessage.m_sData;
        }
        /* Sending the message */
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, &m_tMessages[unKilobotID]);
    }
    else
    {
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, NULL);
    }
}

void GradientFollowingCALF::KiloLOG()
{
    // std::cout << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
    //           << m_fTimeInSeconds << '\t';
    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds << '\t';
    for (size_t kID = 0; kID < m_vecKilobotsPositions.size(); kID++)
    {
        m_kiloOutput
            // << std::noshowpos
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            << (kID < socialRobots ? "soc" : "env") << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsLightSensors[kID] << '\t';
    }
    m_kiloOutput << std::endl;
}

/****************************************/
/****************************************/

void GradientFollowingCALF::PostExperiment()
{
    std::cout << "num robots: " << m_tKilobotEntities.size() << std::endl;
    std::cout << "num social robots: " << socialRobots << std::endl;
    std::cout << "exp length: " << m_fTimeInSeconds << std::endl;
    std::cout << "Overall gradient:" << (overall_gradient / m_tKilobotEntities.size()) / internal_counter << std::endl;
}

/****************************************/
/****************************************/
CColor GradientFollowingCALF::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    Real fPositionX(vec_position_on_plane.GetX()), fPositionY(vec_position_on_plane.GetY());
    CColor cColor = CColor::WHITE;

    CVector2 gradient_pos = CVector2(0.0, 0.0);
    Real max_distance = gradient_radius;

    Real fDistance = Distance(vec_position_on_plane, gradient_pos);

    switch (background_flag)
    {
    case CONTINUOUS:
        if (fDistance < max_distance)
        {
            Real col = 255.0 * (fDistance / max_distance);
            cColor = CColor(col, col, col, 1);
        }
        break;

    case DISCRETE:
        
        switch (discret_bits)
        {
        case BITS_2:
            if (fDistance <= max_distance * 1.0 / 2.0)
            {
                cColor = CColor::BLACK;
            }
            break;

        case BITS_3:
            if (fDistance <= max_distance * 2.0 / 3.0 && fDistance > max_distance * 1.0 / 3.0)
            {
                cColor = CColor::GRAY30;
            }
            if (fDistance <= max_distance * 1.0 / 3.0)
            {
                cColor = CColor::BLACK;
            }
            break;

        case BITS_4:
            if (fDistance <= max_distance * 3.0 / 4.0 && fDistance > max_distance * 2.0 / 4.0)
            {
                cColor = CColor::GRAY60;
            }

            if (fDistance <= max_distance * 2.0 / 4.0 && fDistance > max_distance * 1.0 / 4.0)
            {
                cColor = CColor::GRAY30;
            }

            else if (fDistance <= max_distance * 1.0 / 4.0)
            {
                cColor = CColor::BLACK;
            }
            break;

        default:
            break;
        }



        
        break;

    case NO_BACKGROUND:
    default:
        break;
    }


    /****************************Boarder for wall avoidance*************************************************************************************************/
    // Top border for wall avoidance
    if (vec_position_on_plane.GetY() < vDistance_threshold + 0.005 && vec_position_on_plane.GetY() > vDistance_threshold - 0.005)
    {
        cColor = CColor::ORANGE;
    }
    // Bottom border for wall avoidance
    if (vec_position_on_plane.GetY() < -1.0 * (vDistance_threshold - 0.005) && vec_position_on_plane.GetY() > -1 * (vDistance_threshold + 0.005))
    {
        cColor = CColor::ORANGE;
    }
    // Right border for wall avoidance
    if (vec_position_on_plane.GetX() < vDistance_threshold + 0.005 && vec_position_on_plane.GetX() > vDistance_threshold - 0.005)
    {
        cColor = CColor::ORANGE;
    }
    // Left border for wall avoidance
    if (vec_position_on_plane.GetX() < -1.0 * (vDistance_threshold - 0.005) && vec_position_on_plane.GetX() > -1 * (vDistance_threshold + 0.005))
    {
        cColor = CColor::ORANGE;
    }

    /****************************Communication range*************************************************************************************************/
    // for(auto kID : m_vecKilobotsPositions)
    // {
    //     Real fKiloVision = Distance(vec_position_on_plane, kID);
    //     if (fKiloVision < 0.1 && fKiloVision > 0.09)
    //     {
    //         // cColor = CColor(0, 0, 125, 0);
    //         cColor = CColor::ORANGE;
    //     }
    // }

    return cColor;
}

REGISTER_LOOP_FUNCTIONS(GradientFollowingCALF, "ALF_gradientFollowing_loop_function")
