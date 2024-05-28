#include <visp/vpHomogeneousMatrix.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>

vpHomogeneousMatrix vpHomogeneousMatrixFromGeometryPose(geometry_msgs::PoseStamped geoPose);
vpQuaternionVector lerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t);

template<typename T>
class Logspace {
private:
    T curValue, base;

public:
    Logspace(T first, T base) : curValue(first), base(base) {}

    T operator()() {
        T retval = curValue;
        curValue *= base;
        return retval;
    }
};
std::vector<double> pyLogspace(double start, double stop, int num = 50, double base = 10) {
    double realStart = pow(base, start);
    double realBase = pow(base, (stop-start)/num);

    std::vector<double> retval;
    retval.reserve(num);
    std::generate_n(std::back_inserter(retval), num, Logspace<double>(realStart,realBase));
    return retval;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "evs_reference_trajectory");
    ros::NodeHandle n("~");

    bool verbose;
    std::string pathToInputBag, pathToOutputBag;
    std::string currentPoseTopic, desiredPoseTopic;

    n.param("/evs_genVSRefTraj/verbose", verbose, false);
    n.param<std::string>("/evs_genVSRefTraj/pathToInputBag", pathToInputBag, "");
    n.param<std::string>("/evs_genVSRefTraj/pathToOutputBag", pathToOutputBag, "vs_reference.bag");
    n.param<std::string>("/evs_genVSRefTraj/currentPoseTopic", currentPoseTopic, "/camera/current_pose");
    n.param<std::string>("/evs_genVSRefTraj/desiredPoseTopic", desiredPoseTopic, "/camera/desired_pose");


    //// Read the trajectory data from ros bag
    int nbIteration;
    geometry_msgs::PoseStamped initialPose, desiredPose;
    std::vector<geometry_msgs::PoseStamped> traj;
    rosbag::Bag bag_in(pathToInputBag,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(currentPoseTopic));
    topics.push_back(std::string(desiredPoseTopic));
    rosbag::View view(bag_in,rosbag::TopicQuery(topics));
    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
        if(std::strcmp(m.getTopic().c_str(), topics[0].c_str())==0){
            traj.push_back(*m.instantiate<geometry_msgs::PoseStamped>());
        }else if(std::strcmp(m.getTopic().c_str(), topics[1].c_str())==0){
            desiredPose = *m.instantiate<geometry_msgs::PoseStamped>();
        }
    }
    nbIteration = traj.size();
    initialPose = traj[0];

    //// Compute the reference trajectory
    rosbag::Bag bag_out(pathToOutputBag,rosbag::bagmode::Write);
    geometry_msgs::PoseStamped optimalCurrentPose, currentPose;
    vpHomogeneousMatrix desiredM, optimalCurrentM, initialM;
    vpTranslationVector desiredt, optimalCurrentt, initialt;
    vpQuaternionVector desiredq, optimalCurrentq, initialq;
    desiredM = vpHomogeneousMatrixFromGeometryPose(desiredPose);
    initialM = vpHomogeneousMatrixFromGeometryPose(initialPose);
    desiredt = desiredM.getTranslationVector();
    initialt = initialM.getTranslationVector();
    desiredq = vpQuaternionVector(desiredM.getRotationMatrix());
    initialq = vpQuaternionVector(initialM.getRotationMatrix());

    std::vector<double> logInter;
    logInter = pyLogspace(0,1,nbIteration,10);
    std::reverse(logInter.begin(), logInter.end());
    double maxLog = logInter[0];
    for(int i=0;i<nbIteration;i++){
        logInter[i] = (logInter[i]-maxLog)/(1-maxLog);
    }

    ros::Time t0, t;
    for(int i=0;i<nbIteration;i++){
        currentPose = traj[i];
        if(i==0){
            t0 = currentPose.header.stamp;
            t = ros::TIME_MIN;
        }else{
            ros::Duration d = currentPose.header.stamp - t0;
            t = ros::Time(0,0) + d;
        }

        optimalCurrentt = initialt + ((desiredt - initialt) * logInter[i]);
        optimalCurrentq = lerp(initialq, desiredq, logInter[i]);

        optimalCurrentPose.header = currentPose.header;
        optimalCurrentPose.header.stamp = t;
        optimalCurrentPose.pose.position.x = optimalCurrentt[0];
        optimalCurrentPose.pose.position.y = optimalCurrentt[1];
        optimalCurrentPose.pose.position.z = optimalCurrentt[2];
        optimalCurrentPose.pose.orientation.x = optimalCurrentq.x();
        optimalCurrentPose.pose.orientation.y = optimalCurrentq.y();
        optimalCurrentPose.pose.orientation.z = optimalCurrentq.z();
        optimalCurrentPose.pose.orientation.w = optimalCurrentq.w();

        currentPose.header.stamp = t;

        bag_out.write("/vs/reference", ros::Time(optimalCurrentPose.header.stamp), optimalCurrentPose);
        bag_out.write("/vs/current", ros::Time(optimalCurrentPose.header.stamp), currentPose);
    }

    bag_out.close();

    return 0;
}

vpHomogeneousMatrix vpHomogeneousMatrixFromGeometryPose(geometry_msgs::PoseStamped geoPose){
    vpPoseVector vpPose;
    vpHomogeneousMatrix M;
    vpTranslationVector t;
    vpQuaternionVector q;
    t[0] = geoPose.pose.position.x;
    t[1] = geoPose.pose.position.y;
    t[2] = geoPose.pose.position.z;
    q.set(geoPose.pose.orientation.x, geoPose.pose.orientation.y, geoPose.pose.orientation.z, geoPose.pose.orientation.w);
    M.buildFrom(t, vpRotationMatrix(q));
    return M;
}


vpQuaternionVector lerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t){
    assert(t >= 0 && t <= 1);

    double cosHalfTheta = q0.x() * q1.x() + q0.y() * q1.y() + q0.z() * q1.z() + q0.w() * q1.w();
    vpQuaternionVector q1_ = q1;
    if (cosHalfTheta < 0) {
        cosHalfTheta = -cosHalfTheta;
        q1_ = -q1;
    }

    vpQuaternionVector qLerp;
    qLerp.set(q0.x() - t * (q0.x() - q1.x()),
              q0.y() - t * (q0.y() - q1.y()),
              q0.z() - t * (q0.z() - q1.z()),
              q0.w() - t * (q0.w() - q1.w()));

    qLerp.normalize();

    return qLerp;
}

