namespace robots { 
    class Kinematics { 
        public: int num_of_joints, num_free_parameters; 
        Kinematics(); 
        ~Kinematics(); 
        std::vector<float> forward(std::vector<float> joint_config);
        std::vector<float> inverse(std::vector<float> ee_pose, std::vector<float> free_vals);
        std::vector<float> inverse(std::vector<float> ee_pose, unsigned int free_itr, double free_min, double free_max);
    }; 
}