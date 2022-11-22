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

    // wall avoidance params
    const CVector2 up_direction(0.0, -1.0);
    const CVector2 down_direction(0.0, 1.0);
    const CVector2 left_direction(1.0, 0.0);
    const CVector2 right_direction(-1.0, 0.0);
    const int kProximity_bits = 8;

    // log time counter
    int internal_counter = 0;

}

/****************************************/
/****************************************/

GradientFollowingCALF::GradientFollowingCALF() : m_unDataAcquisitionFrequency(10)
{
    c_rng = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void GradientFollowingCALF::Init(TConfigurationNode &t_node)
{


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
}

/****************************************/
/****************************************/

void GradientFollowingCALF::SetupInitialKilobotStates()
{
    /* Resize variables related to the number of Kilobots */
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsLightSensors.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    /* Setup the min time between two message sent to a kilobot (ARK message sending limits)*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        /* Setup the virtual states of a kilobot*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
}

/****************************************/
/****************************************/

void GradientFollowingCALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    /* Get the robot ID */
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
}

/****************************************/
/****************************************/

void GradientFollowingCALF::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    std::cout << "Arena size: " << vArena_size << "\tWA threshold: " << vDistance_threshold << "\n";
}

/****************************************/
/****************************************/

void GradientFollowingCALF::GetExperimentVariables(TConfigurationNode &t_tree)
{
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");

    // /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "kilo_filename", m_strKiloOutputFileName);
    std::cout<< "Filename: " << m_strKiloOutputFileName << std::endl;

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
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsLightSensors[unKilobotID] = GetFloorColor(m_vecKilobotsPositions[unKilobotID]);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

/****************************************/
/****************************************/

void GradientFollowingCALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    /* Create ARK-type messages variables */
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;

    /* Flag for existance of message to send */
    bool bMessageToSend = true;

    /* Get the kilobot ID and state (Position and Orientation in this example*/
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    tKilobotMessage.m_sID = unKilobotID;
    tKilobotMessage.m_sData = 0;

    if (m_vecKilobotsLightSensors[unKilobotID] == CColor::BLACK)
    {
        tKilobotMessage.m_sType = kBLACK;
    }
    else if (m_vecKilobotsLightSensors[unKilobotID] == CColor::GRAY30)
    {
        tKilobotMessage.m_sType = kGRAY;
    }
    else if (m_vecKilobotsLightSensors[unKilobotID] == CColor::WHITE)
    {
        tKilobotMessage.m_sType = kWHITE;
    }
    else if (m_vecKilobotsLightSensors[unKilobotID] != CColor::ORANGE)
    {
        std::cout << "Error, wrong floor colour " << m_vecKilobotsLightSensors[unKilobotID] << "\n";
    }

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
    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds << '\t';
    size_t kID = 0;
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        // argos::LOG<< kID <<std::endl;
        // argos::LOG<< m_vecKilobotsPositions[kID].GetX() <<std::endl;
        // argos::LOG<< m_vecKilobotsPositions[kID].GetY() <<std::endl;
        m_kiloOutput
            // << kID << '\t'
            // << m_vecKilobotStates_ALF[kID] << '\t' //TODO: this should be the colour, but for now is the state
            // << m_vecKilobotsPositions[kID].GetX() << '\t'
            // << m_vecKilobotsPositions[kID].GetY() << '\t'
            // << m_vecKilobotsOrientations[kID] << '\t'
            // << m_vecKilobotStates_ALF[kID];

            // << std::noshowpos
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << GetKilobotPosition(*m_tKilobotEntities[it]).GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << GetKilobotPosition(*m_tKilobotEntities[it]).GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << GetKilobotOrientation(*m_tKilobotEntities[it]).GetValue() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << GetFloorColor(GetKilobotPosition(*m_tKilobotEntities[it])) << '\t';
            kID++;
    }
    m_kiloOutput << std::endl;
}

/****************************************/
/****************************************/

CColor GradientFollowingCALF::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    Real fPositionX(vec_position_on_plane.GetX()), fPositionY(vec_position_on_plane.GetY());
    CColor cColor = CColor::WHITE;

    CVector2 gradient_pos = CVector2(0.0, 0.0);
    Real max_distance = vArena_size / 2.0;

    Real fDistance = Distance(vec_position_on_plane, gradient_pos);
    if (fDistance <= max_distance * 2.0 / 3.0 && fDistance > max_distance * 1.0 / 3.0)
    {
        cColor = CColor::GRAY30;
    }

    else if (fDistance <= max_distance * 1.0 / 3.0)
    {
        cColor = CColor::BLACK;
    }

    /**Boarder for wall avoidance*/
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

    // Real fKiloVision = Distance(vec_position_on_plane, m_vecKilobotsPositions[6]);
    // if (fKiloVision < 0.05 && fKiloVision > 0.04)
    // {
    //     // cColor = CColor(0, 0, 125, 0);
    //     cColor = CColor::ORANGE;
    // }

    return cColor;
}

REGISTER_LOOP_FUNCTIONS(GradientFollowingCALF, "ALF_gradientFollowing_loop_function")
