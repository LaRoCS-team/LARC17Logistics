#include "navigation_fuzzy/Desvia.hpp"

#define N_SENSORS 9
#define MAX_SQUARE_RADIUS_INT 40
#define MAX_SQUARE_RADIUS_SIDES 30

using namespace fl;

std::vector <std::pair <float,float> > pointsDetected(N_SENSORS);

float squareDistance(std::pair <float,float> point) {
    return std::sqrt((std::pow(point.first,2) + std::pow(point.second,2)));
}

void Desvia::print(const std::string str) {
        std::cout << str << std::endl;
}

void Desvia::distanceSensorsCallback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    for (int i=0; i<N_SENSORS; i++) {
        pointsDetected[i] = std::make_pair(msg->points[i].x, msg->points[i].y);
    }
}

/*
void Desvia::executeCB(const fuzzy::FuzzyGoalConstPtr &goal) {
    float intEsqS = 100*squareDistance(pointsDetected[1]);
    float esqS = 100*squareDistance(pointsDetected[2]);
    float dirS = 100*squareDistance(pointsDetected[7]);
    float intDirS = 100*squareDistance(pointsDetected[8]);
    bool success = true;

    while ((intEsqS > 0 && intEsqS < MAX_SQUARE_RADIUS_INT) ||  (esqS > 0 && esqS < MAX_SQUARE_RADIUS_SIDES) || (intDirS > 0 && intDirS < MAX_SQUARE_RADIUS_INT) || (dirS > 0 && dirS < MAX_SQUARE_RADIUS_SIDES)) {
        using namespace fl;
        std::string status;
        geometry_msgs::Twist tw;

        ros::Rate r(100);

        if (as_.isPreemptRequested() || !ros::ok())  {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }

        if (not engine->isReady(&status))
            throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

        tw.linear.x = 0;
        tw.linear.y = 0;
        tw.linear.z = 0;
        tw.angular.x = 0;
        tw.angular.y = 0;
        tw.angular.z = 0;

        intEsqS = 100*squareDistance(pointsDetected[1]);
        esqS = 100*squareDistance(pointsDetected[2]);
        dirS = 100*squareDistance(pointsDetected[7]);
        intDirS = 100*squareDistance(pointsDetected[8]);

        this->intDir->setValue(intDirS);
        this->dir->setValue(dirS);
        this->intEsq->setValue(intEsqS);
        this->esq->setValue(esqS);
        engine->process();
        tw.linear.x = this->linX->getValue();
        tw.linear.y = this->linY->getValue();
        tw.angular.z = this->angular->getValue();
        cout << "linX:" << tw.linear.x <<" linY:" << tw.linear.y <<" ang:" << tw.angular.z << "\n";

        feedback_.sensors = true;
        as_.publishFeedback(feedback_);

        pubTwistMsg.publish(tw);
    }

        result_.res = success;
        as_.setSucceeded(result_);
}
*/

void Desvia::goalCB() {
    goal_ = as_.acceptNewGoal()->order;
    print("New goal set.");
    std::cout << "goal is " << goal_ << std::endl;
}

void Desvia::preemptCB() {
    print(action_name_ + ": Preempted");
    // set the action state to preempted
    as_.setPreempted();
}

void Desvia::spin() {
    ros::Rate lr(node_loop_rate);
    while(nh.ok()) {
		    if(!as_.isActive()) {
			      //std::cout <<"No fuzzy goal active" << std::endl;
		    }
		    else {
            std::cout << "fuzzy active!" << std::endl;

            float intEsqS = 100*squareDistance(pointsDetected[1]);
            float esqS = 100*squareDistance(pointsDetected[2]);
            float dirS = 100*squareDistance(pointsDetected[7]);
            float intDirS = 100*squareDistance(pointsDetected[8]);

            if ((intEsqS > 0 && intEsqS < MAX_SQUARE_RADIUS_INT) ||  (esqS > 0 && esqS < MAX_SQUARE_RADIUS_SIDES) || (intDirS > 0 && intDirS < MAX_SQUARE_RADIUS_INT) || (dirS > 0 && dirS < MAX_SQUARE_RADIUS_SIDES)) {
                using namespace fl;
                std::string status;
                geometry_msgs::Twist tw;

                if (not engine->isReady(&status))
                    throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

                tw.linear.x = 0;
                tw.linear.y = 0;
                tw.linear.z = 0;
                tw.angular.x = 0;
                tw.angular.y = 0;
                tw.angular.z = 0;

                this->intDir->setValue(intDirS);
                this->dir->setValue(dirS);
                this->intEsq->setValue(intEsqS);
                this->esq->setValue(esqS);
                engine->process();
                tw.linear.x = this->linX->getValue();
                tw.linear.y = this->linY->getValue();
                tw.angular.z = this->angular->getValue();
                cout << "linX:" << tw.linear.x <<" linY:" << tw.linear.y <<" ang:" << tw.angular.z << "\n";

                /*feedback_.feed = true;
                as_.publishFeedback(feedback_);*/

                pubTwistMsg.publish(tw);
            } else {
                result_.res = true;
                std::cout << "fuzzy finished!" << std::endl;
                as_.setSucceeded(result_);
            }
        }
		    lr.sleep();
		    ros::spinOnce();
    }
}


Desvia::Desvia(std::string name) :
    as_(nh, name, false),
    action_name_(name),
    node_loop_rate(20) {
	  //time(&this->name);
    //std::cout << "Name at construct " << this->name << "/n";
    //ros::NodeHandle nh;
    this->getDistanceSensors = nh.subscribe("distance_sensors", 1, &Desvia::distanceSensorsCallback, this);
    pubTwistMsg = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    as_.registerGoalCallback(boost::bind(&Desvia::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Desvia::preemptCB, this));

    std_msgs::Bool aux;
    aux.data = false;

    //init fuzzy
    using namespace fl;

    this->engine->setName("");
    this->engine->setDescription("");

    this->intDir->setName("intDir");
    this->intDir->setDescription("");
    this->intDir->setEnabled(true);
    this->intDir->setRange(0.000, 60.000);
    this->intDir->setLockValueInRange(false);
    this->intDir->addTerm(new Trapezoid("muitoPertoID", 0, 0, 20, 30));
    this->intDir->addTerm(new Triangle("pertoID", 22.5, 30, 40));
    this->engine->addInputVariable(this->intDir);

    this->dir->setName("dir");
    this->dir->setDescription("");
    this->dir->setEnabled(true);
    this->dir->setRange(0.000, 60.000);
    this->dir->setLockValueInRange(false);
    this->dir->addTerm(new Trapezoid("muitoPertoD", 0, 0, 20, 30));
    this->engine->addInputVariable(this->dir);

    this->intEsq->setName("intEsq");
    this->intEsq->setDescription("");
    this->intEsq->setEnabled(true);
    this->intEsq->setRange(0.000, 60.000);
    this->intEsq->setLockValueInRange(false);
    this->intEsq->addTerm(new Trapezoid("muitoPertoIE", 0, 0, 20, 30));
    this->intEsq->addTerm(new Triangle("pertoIE", 22.5, 30, 40));
    this->engine->addInputVariable(this->intEsq);

    this->esq->setName("esq");
    this->esq->setDescription("");
    this->esq->setEnabled(true);
    this->esq->setRange(0.000, 60.000);
    this->esq->setLockValueInRange(false);
    this->esq->addTerm(new Trapezoid("muitoPertoE", 0, 0, 20, 30));
    this->engine->addInputVariable(this->esq);

    this->linX->setName("linX");
    this->linX->setDescription("");
    this->linX->setEnabled(true);
    this->linX->setRange(-1, 1);
    this->linX->setLockValueInRange(false);
    this->linX->setAggregation(new Maximum);
    this->linX->setDefuzzifier(new Centroid(100));
    this->linX->setDefaultValue(0);
    this->linX->setLockPreviousValue(false);
    this->linX->addTerm(new Triangle("lentoX", 0, 0.05, 0.1));
    this->linX->addTerm(new Triangle("rapidoX", 0.05, 0.1, 0.15));
    this->engine->addOutputVariable(this->linX);

    this->linY->setName("linY");
    this->linY->setDescription("");
    this->linY->setEnabled(true);
    this->linY->setRange(-1, 1);
    this->linY->setLockValueInRange(false);
    this->linY->setAggregation(new Maximum);
    this->linY->setDefuzzifier(new Centroid(100));
    this->linY->setDefaultValue(0);
    this->linY->addTerm(new Triangle("dRapido", -0.3, -0.3, -0.1));
    this->linY->addTerm(new Triangle("eRapido", 0.1, 0.3, 0.3));
    this->engine->addOutputVariable(this->linY);

    this->angular->setName("angular");
    this->angular->setDescription("");
    this->angular->setEnabled(true);
    this->angular->setRange(-1, 1);
    this->angular->setLockValueInRange(false);
    this->angular->setAggregation(new Maximum);
    this->angular->setDefuzzifier(new Centroid(100));
    this->angular->setDefaultValue(0);
    this->angular->setLockPreviousValue(false);
    this->angular->addTerm(new Trapezoid("girarDRapido",-0.8, -0.7, -0.5, -0.4));
    this->angular->addTerm(new Trapezoid("girarDLento", -0.5, -0.4 , -0.3, -0.2));
    this->angular->addTerm(new Trapezoid("girarELento", 0.2, 0.3, 0.4, 0.5));
    this->angular->addTerm(new Trapezoid("girarERapido", 0.4, 0.5, 0.7, 0.8));
    this->engine->addOutputVariable(this->angular);

    this->mamdani->setName("mamdani");
    this->mamdani->setDescription("");
    this->mamdani->setEnabled(true);
    this->mamdani->setConjunction(new Minimum);
    this->mamdani->setDisjunction(new Maximum);
    this->mamdani->setImplication(new AlgebraicProduct);
    this->mamdani->setActivation(new General);

    this->mamdani->addRule(Rule::parse("if intDir is muitoPertoID then linX is lentoX", this->engine));
    this->mamdani->addRule(Rule::parse("if intDir is muitoPertoID then angular is girarERapido", this->engine));

    this->mamdani->addRule(Rule::parse("if intDir is pertoID then linX is rapidoX", this->engine));
    this->mamdani->addRule(Rule::parse("if intDir is pertoID then angular is girarELento", this->engine));

    this->mamdani->addRule(Rule::parse("if dir is muitoPertoD then linY is eRapido", this->engine));

    this->mamdani->addRule(Rule::parse("if intEsq is muitoPertoIE then linX is lentoX", this->engine));
    this->mamdani->addRule(Rule::parse("if intEsq is muitoPertoIE then angular is girarDRapido", this->engine));

    this->mamdani->addRule(Rule::parse("if intEsq is pertoIE then linX is rapidoX", this->engine));
    this->mamdani->addRule(Rule::parse("if intEsq is pertoIE then angular is girarDLento", this->engine));

    this->mamdani->addRule(Rule::parse("if esq is muitoPertoE then linY is dRapido", this->engine));
    this->engine->addRuleBlock(this->mamdani);

    as_.start();
}
