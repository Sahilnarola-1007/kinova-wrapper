#include"kinova_interface/KinovaInterface.hpp"
#include<iostream>
#include<stdexcept>

namespace kinova_wrapper{

    KinovaInterface::KinovaInterface(){
        std::cout<<"KinovaInterface created. Call connect method \
        to build TCP connection"<< std::endl;
    };
    
    KinovaInterface::~KinovaInterface(){

        std::cout<<"Destroying connection and reseting all the objects"\
        <<std::endl;
        disconnect();

    };

// =============================================================================
// Part 1: Connection and disconnection set up
// =============================================================================
   
//fun 1: connect() 
bool KinovaInterface::connect(const std::string& ip_address,
                                uint32_t port,
                                const std::string &username,
                                const std::string &password
        ){
                std::lock_guard<std::mutex> lock(mutex_); 
                
                //step 1: To check the connection if its already connected or not
                if(connected_.load()){
                    std::cout<<"Already connected, Closing a session and connection"<<std::endl;
                    disconnectLocked();
                    }

                // step 2: Creating transport client          
                try{  
                transport_=std::make_unique<k_api::TransportClientTcp>();
                transport_->connect(ip_address,port);
                }
                catch(const std::exception & e)
                {
                    std::cout<<"Robot unreachable with ip: "<<ip_address
                    <<" and port "<<port<<std::endl;
                    std::cerr<<e.what()<<std::endl;
                    transport_.reset();
                    return false;
                }
                
                //step 3: creating router client
                router_=std::make_unique<k_api::RouterClient>(transport_.get());
                
                //step 4: Creating session_manager for managing session
                session_manager_=std::make_unique<k_api::SessionManager>(router_.get());
                
                //step 5: Creating session info and pass it to the session manager
                k_api::CreateSessionInfo session_info_;
                session_info_.username=username;
                session_info_.password=password;
                
                //step 6: starting a session
                try{
                    session_manager_->CreateSession(session_info_);
                    std::cout<<"session created"<<std::endl;
                }

                catch(const std::exception &e)
                {
                    std::cout<<"session not created. Authentication failed!"<<std::endl;
                    std::cout<<e.what()<<std::endl;
                    
                    session_manager_.reset();
                    router_.reset();
                    transport_->disconnect();
                    transport_.reset();
                    return false;
                }

                // step 7: Creating the base_client which will talk with the robot's base
                try{
                    base_client_=std::make_unique<k_api::Base::BaseClient>(router_.get());
                    std::cout<<"Base Client created"<<std::endl;
                }
                catch(const std::exception &e){
                    std::cout<<"Base client is not created"<<std::endl;
                    std::cout<<e.what()<<std::endl;
                    
                    base_client_.reset();
                    session_manager_->CloseSession();
                    session_manager_.reset();
                    router_.reset();
                    transport_->disconnect();
                    transport_.reset();
                    return false;
                }
                
                // step 8: Calling GetJointLimits method to get the joint limits
                try{

                    std::cout<<"fetching the joint limits..."<<std::endl;
                    auto joints_limits=base_client_->GetJointLimits();  // It has min and max limits of all the joints
                    
                    joint_max_limits_.clear();
                    joint_min_limits_.clear();
                    joint_max_limits_.reserve(kNumJoints); // Pre allocation of the memory
                    joint_min_limits_.reserve(kNumJoints); 

                    for (const auto& limit:joints_limits)
                    {
                        joint_max_limits_.push_back(limit.max_value);
                        joint_min_limits_.push_back(limit.min_value);
                    }

                    std::cout<<"Fetched joint limits!"<<std::endl;
                    
                    std::cout<<"maximum limit for all joints:= ";
                    for(const double &joint:joint_max_limits_){
                        std::cout<<joint<<" ";
                    }
                    std::cout<<"\n";

                    std::cout<<"minimum limit for all joints:= ";
                    for(const double &joint:joint_min_limits_){
                        std::cout<<joint<<" ";
                    }
                    std::cout<<"\n";
                    }
                    
                catch(const std::exception &e){
                        std::cout<<"Did not get joints limits"<<std::endl;
                        std::cout<<e.what()<<std::endl;
                    
                    base_client_.reset();
                    session_manager_->CloseSession();
                    session_manager_.reset();
                    router_.reset();
                    transport_->disconnect();
                    transport_.reset();
                    return false;

                }
                connected_.store(true);
                e_stop_active_.store(false);
                std::cout << " Connected successfully and ready.\n";
                return true;

        };

//fun 2: disconnectLocked()
void KinovaInterface::disconnectLocked() {
            
            //If not connected or nothing to clean up then return to the caller
            if(!connected_.load())
            {std::cout<<"Disconnected"<<std::endl;
            return;}

            std::cout<<"disconnecting"<<std::endl;
          base_client_.reset();  
          try{
            if(session_manager_)
            {session_manager_->CloseSession();}
            }
            
            catch(const std::exception &e)
            {
                std::cout<<"Warning! Error while closing a session"<<std::endl;
                std::cout<<" continue cleaning up.."<<std::endl;
                std::cerr<<e.what()<<std::endl;
                   
            }
            
            session_manager_.reset();
            router_.reset();
            
            try {
            if (transport_) {
                transport_->disconnect();
                }
            } 
            
            catch (const std::exception& e) 
            { std::cerr << " Warning: transport disconnect failed: "
                  << e.what() << "\n";
            }
            
            transport_.reset();
            
            joint_max_limits_.clear();
            joint_min_limits_.clear();

            connected_.store(false);
            }

//fun 3: disconnect()
void KinovaInterface::disconnect()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            disconnectLocked();
        }  

//fun 4: isConnected()     
bool KinovaInterface::isConnected() const {
                return connected_.load();
    
        }
// =============================================================================
// Part 2: Motion Commands
// =============================================================================
 
// fun1: validateJointAngles 
// Called under mutex_ — validates angles are within firmware limits (degrees internally)
bool KinovaInterface::validateJointAngles(const std::vector<double>&angles ) const{
    
    if(angles.size()!=static_cast<size_t>(kNumJoints)){
        std::cerr<<"Error! The number of joints are: "<<angles.size()
        <<" but expected "<<kNumJoints<<std::endl;
        return false;
        }
    for (size_t i=0;i<angles.size();i++){

        //Converting angles Rad to Deg because kinova expects angles in Deg
        double angle_deg=angles[i]*kRadToDeg;
    
        if(angle_deg<joint_min_limits_[i] || angle_deg>joint_max_limits_[i]){
            std::cerr<<" Error! joint:= "<<i<<"is not within the range"
            <<std::endl;

            return false;
        }
    }
    return true;
}

//fun 2:moveToJointAngles (sync version): Blocking method
bool KinovaInterface::moveToJointAngles(const std::vector<double> &angles){

    // To check the status of atomic variables before lock 
    if(!connected_.load()){
        std::cerr<<"Error! Not connected"<<std::endl;
        return false;
    }
    if(e_stop_active_.load()){
        std::cerr<<"Error! e-stop is active we can't proceed this action"
        <<std::endl;
        return false;
    }

    // thread-safe lock to protect shared state(joint_min_limits_ and joint_max_limit_ in the validate joint function)
    std::lock_guard<std::mutex> lock(mutex_);

    //Validating joint angles
    if (!validateJointAngles(angles)){
        return false;
    }
    try{

        // Building kortex action: To send the commands to the joints
        k_api::Base::Action action;
        action.is_joint_action=true;

        for(size_t i=0; i< angles.size();i++)
        {
            k_api::Base::JointAngle joint;
            joint.joint_identifier=static_cast<uint32_t>(i);
            joint.value=angles[i]*kRadToDeg;
            action.target_joint_angles.joint_angles.push_back(joint);
        }

        //Calling kortex
        base_client_->ExecuteAction(action);
        std::cout<<"joint motion completed"<<std::endl;

        return true;    
        }
        
        catch(const std::exception &e){
            std::cerr<<"Error! Joint motion failed"<<
            e.what()<<std::endl;
            
            return false;
        }
    }

//fun 3: moveTojointAnglesAysnc()
std::future<bool> KinovaInterface::moveToJointAnglesAsync(
    const std::vector<double> &angles){

    // Capture angles BY VALUE into the lambda.
    // Why? The caller's vector might go out of scope before the async thread runs.
    // Capturing by reference would be a dangling reference → undefined behavior.

    return std::async(std::launch::async,[this,angles](){
        return this->moveToJointAngles(angles);
    });

}

//fun 4: moveToCartesianPose
bool KinovaInterface::moveToCartesianPose(const Pose& pose){
    
    if(!connected_.load()){
        std::cerr<<"Error! Not connected"<<std::endl;
        return false;
    }
    if(e_stop_active_.load()){
        std::cerr<<"Error! e-stop is active we can't proceed this action"
        <<std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    try{

        k_api::Base::Action action; 
        action.is_cartesian_action=true;
        action.target_pose.x=pose.x;
        action.target_pose.y = pose.y;
        action.target_pose.z = pose.z;
        action.target_pose.theta_x = pose.theta_x;
        action.target_pose.theta_y = pose.theta_y;
        action.target_pose.theta_z = pose.theta_z;
 
        //Calling kortex
        base_client_->ExecuteAction(action);
 
        std::cout << "Cartesian motion completed.\n";
        return true;
        } 
        
    catch (const std::exception& e) 
        {
        std::cerr << "Cartesian motion failed: "
                  << e.what() << "\n";
        return false;
        }


    }

//fun 5:moveToCartesianPose(Async version)
std::future<bool> KinovaInterface::moveToCartesianPoseAsync
    (const Pose &pose)
    {
        return std::async(std::launch::async,
            [this,pose](){
            return this->moveToCartesianPose(pose);
        
        });
    }

//fun 6: emergency stop
void KinovaInterface::emergencyStop(){
    e_stop_active_.store(true);

    std::lock_guard<std::mutex> lock(mutex_);
    
    try{
         
        if(base_client_){
        base_client_->ApplyEmergencyStop();
        std::cout<<"Emergency stop is activated! kortex call succeed"<<std::endl;
            }
        }
    catch(const std::exception &e){

        std::cerr<<"Emergency stop kortex call failed: "
        <<e.what()<<"- Flag still set "<<std::endl;
        }
        }

//fun 7: Clear emergency stop 
bool KinovaInterface::clearEmergencyStop(){
        std::lock_guard<std::mutex> lock(mutex_);
        try{

            if(base_client_){
                base_client_->ClearFaults(); //clears the state
            }
        
            e_stop_active_.store(false);
            std::cout<<"Emergency stop is cleared! ready to work"
            <<std::endl;

            return true;
        }
        
        catch(const std::exception &e){
            std::cerr<<"Failed to clear the emergency stop"
            <<e.what();   
            
            //still emergency stop=true safer to stay stopped

            return false;
        }


}

// =============================================================================
// Part 3: state reading
// =============================================================================

//fun 1: Reading current joint angles
std::vector<double> KinovaInterface::getJointAngles(){
    
    if(!connected_.load()){
        std::cerr<<"Not connected! Joint angle reading failed"<<std::endl;
        return {};
    }
    
    std::lock_guard<std::mutex> lock(mutex_);

    try{
            auto joint_angles=base_client_->GetMeasuredJointAngles();
            
            std::vector<double> angles;
            angles.reserve(kNumJoints);

            for(const auto& angle:joint_angles.joint_angles ){
                
                angles.push_back(angle.value*kDegToRad);
            }
            return angles; // Angles in radians
        
    }
    catch(const std::exception &e){
        std::cerr<<"Joint angles reading failed: "<<
        e.what()<<std::endl;
        return {};
    }
    
}

// fun 2: Reading current pose 
Pose KinovaInterface::getCurrentPose(){
    
    if(!connected_.load()){
        std::cerr<<"Not connected! Pose reading failed"<<std::endl;
        return Pose{};
    }

    std::lock_guard<std::mutex> lock(mutex_);

    try{
        
            auto kortex_pose=base_client_->GetMeasuredCartesianPose();
            Pose pose;
        
            pose.x = kortex_pose.x;
            pose.y = kortex_pose.y;
            pose.z = kortex_pose.z;
            pose.theta_x = kortex_pose.theta_x;
            pose.theta_y = kortex_pose.theta_y;
            pose.theta_z = kortex_pose.theta_z;

            return pose;
        }

    catch(const std::exception &e){
        std::cerr<<"Pose reading failed: "<<
        e.what()<<std::endl;
        return Pose{};
    }
}

// fun 3: Reading wrench:(fx,fy,fz,tx,ty,tz)
std::vector<double> KinovaInterface::getWrench(){
    
    if(!connected_.load()){
        std::cerr<<"Not connected! Wrench reading failed"<<std::endl;
        return {};
    }

    std::lock_guard<std::mutex> lock(mutex_);

    try{
            auto wrench=base_client_->GetMeasuredWrench();

            return {wrench.force_x,wrench.force_y, wrench.force_z,
            wrench.torque_x, wrench.torque_y, wrench.torque_z
        };

        }

    catch(const std::exception &e){
        std::cerr<<"Wrench reading failed: "<<
        e.what()<<std::endl;
        
        return {};
    }
}

// =============================================================================
// Part 4: setSpeedLimit
// =============================================================================
 
//fun 1: setting speed limit by using fraction
bool KinovaInterface::setSpeedLimit(double fraction){
        
    if(!connected_.load()){
        std::cerr<<"Not connected! Setting speed limit failed"<<std::endl;
        return false;
        } 
    
        if(fraction<0.0 || fraction>1.0){
            std::cerr<<"speed limit is not within the range"<<
            std::endl;
            return false;
        }

    std::lock_guard<std::mutex> lock(mutex_);

        // TODO: call Kortex speed limit API
        
        current_speed_fraction_=fraction;
        std::cout << "Speed limit set to "
        << (fraction * 100.0) << "%.\n";
        return true;
}

}


