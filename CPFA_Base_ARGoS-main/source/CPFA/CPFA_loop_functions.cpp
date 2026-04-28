#include "CPFA_loop_functions.h"
#include <algorithm>
#include <cmath>

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
        SimTime(0),
	//MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()),
    MaxSimTime(0),//qilu 02/05/2021
        CollisionTime(0), 
        lastNumCollectedFood(0),
        currNumCollectedFood(0),
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	UseAHCFA(1),
	FoodDistribution(2),
	FoodItemCount(256),
	PowerlawFoodUnitCount(256),
	NumberOfClusters(4),
	ClusterWidthX(8),
	ClusterWidthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.12),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	NumDistributedFood(0),
	score(0),
	PrintFinalScore(0),
	AdaptiveMaxDepth(4),
	AdaptiveSplitVisitThreshold(6),
	AdaptiveClusterWindowTicks(0),
	AdaptiveClusterHitThreshold(3),
	AdaptiveClaimWindowTicks(0),
	AdaptiveTopCandidates(10),
	AdaptiveExploreWeight(1.00),
	AdaptivePheromoneWeight(0.60),
	AdaptiveResourceWeight(1.25),
	AdaptiveClusterProtectWeight(0.35),
	AdaptiveRandomWeight(0.15)
{}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {	
 
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");

	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	argos::GetNodeAttribute(CPFA_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");
	
	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/05/2021 dyn2d error

	argos::GetNodeAttribute(settings_node, "MaxSimCounter", MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", DrawTargetRays);
	argos::GetNodeAttributeOrDefault(settings_node, "UseAHCFA", UseAHCFA, UseAHCFA);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", FoodDistribution);
	argos::GetNodeAttribute(settings_node, "FoodItemCount", FoodItemCount);
	argos::GetNodeAttribute(settings_node, "PowerlawFoodUnitCount", PowerlawFoodUnitCount);
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterWidthY", ClusterWidthY);
	argos::GetNodeAttribute(settings_node, "FoodRadius", FoodRadius);
    argos::GetNodeAttribute(settings_node, "NestRadius", NestRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", NestElevation);
    argos::GetNodeAttribute(settings_node, "NestPosition", NestPosition);
    FoodRadiusSquared = FoodRadius*FoodRadius;

    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
    }
    else{
        NumDistributedFood = FoodItemCount;  
    }
    

	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

        ArenaWidth = ArenaSize[0];
        
        if(abs(NestPosition.GetX()) < -1) //quad arena
        {
            NestRadius *= sqrt(1 + log(ArenaWidth)/log(2));
        }
        else
        {
            NestRadius *= sqrt(log(ArenaWidth)/log(2));
        }
        argos::LOG<<"NestRadius="<<NestRadius<<endl;
	   // Send a pointer to this loop functions object to each controller.
	   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    Num_robots = footbots.size();
    argos::LOG<<"Number of robots="<<Num_robots<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        c2.SetLoopFunctions(this);
	    }
     
     
   NestRadiusSquared = NestRadius*NestRadius;
	
    SetFoodDistribution();
    AdaptiveClusterWindowTicks = 90 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();
    AdaptiveClaimWindowTicks = 45 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();
    ResetAdaptiveRegions();
  
 ForageList.clear(); 
 last_time_in_minutes=0;
 
}


void CPFA_loop_functions::Reset() {
	   if(VariableFoodPlacement == 0) {
		      RNG->Reset();
	   }

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    MaxSimCounter = SimCounter;
    SimCounter = 0;
    score = 0;
   
    FoodList.clear();
    CollectedFoodList.clear();
    FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
    TargetRayList.clear();
    ResetAdaptiveRegions();
    
    SetFoodDistribution();
    
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;
   
    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    c2.Reset();
    }
}

void CPFA_loop_functions::PreStep() {
    SimTime++;
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    if(curr_time_in_minutes - last_time_in_minutes==1){
		      
        ForageList.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
    }


	   UpdatePheromoneList();

	   if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
        for(size_t i = 0; i < FoodColoringList.size(); i++) {
            FoodColoringList[i] = argos::CColor::BLACK;
        }
	   }
 
    if(FoodList.size() == 0) {
	FidelityList.clear();
	PheromoneList.clear();
        TargetRayList.clear();
    }
}

void CPFA_loop_functions::PostStep() {
	// nothing... yet...
}

bool CPFA_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
		isFinished = true;
	}
    //set to collected 88% food and then stop
    if(score >= NumDistributedFood){
		isFinished = true;
		}
         
         
    
	if(isFinished == true && MaxSimCounter > 1) {
		size_t newSimCounter = SimCounter + 1;
		size_t newMaxSimCounter = MaxSimCounter - 1;
        argos::LOG<< "time out..."<<endl; 
		PostExperiment();
		Reset();

		SimCounter    = newSimCounter;
		MaxSimCounter = newMaxSimCounter;
		isFinished    = false;
	}

	return isFinished;
}

void CPFA_loop_functions::PostExperiment() {
	  
     printf("%f, %f, %lu\n", score, getSimTimeInSeconds(), RandomSeed);
       
                  
    if (PrintFinalScore == 1) {
        string type="";
        if (FoodDistribution == 0) type = "random";
        else if (FoodDistribution == 1) type = "cluster";
        else type = "powerlaw";
            
        ostringstream num_tag;
        num_tag << FoodItemCount; 
              
        ostringstream num_robots;
        num_robots <<  Num_robots;
   
        ostringstream arena_width;
        arena_width << ArenaWidth;
        
        ostringstream quardArena;
        if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
             quardArena << 1;
         }
         else{
             quardArena << 0;
        }
        
        string algorithm = UseAHCFA == 1 ? "AHCFA" : "CPFA";
        string header = "./results/"+ type+"_"+algorithm+"_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_" + quardArena.str() +"_";
       
        //unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("Default").GetInverseSimulationClockTick();
        unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
       
        /* Real total_travel_time=0;
        Real total_search_time=0;
        ofstream travelSearchTimeDataOutput((header+"TravelSearchTimeData.txt").c_str(), ios::app);
        */
        
        
        argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
        for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
            argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
            BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
            CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
            CollisionTime += c2.GetCollisionTime();
            
            /*if(c2.GetStatus() == "SEARCHING"){
                total_search_time += SimTime-c2.GetTravelingTime();
                total_travel_time += c2.GetTravelingTime();
	    }
            else {
		total_search_time += c2.GetSearchingTime();
		total_travel_time += SimTime-c2.GetSearchingTime();
            } */        
        }
        //travelSearchTimeDataOutput<< total_travel_time/ticks_per_second<<", "<<total_search_time/ticks_per_second<<endl;
        //travelSearchTimeDataOutput.close();   
             
        ofstream dataOutput( (header+ "iAntTagData.txt").c_str(), ios::app);
        // output to file
        if(dataOutput.tellp() == 0) {
            dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
        }
    
        //dataOutput <<data.CollisionTime/16.0<<", "<< time_in_minutes << ", " << data.RandomSeed << endl;
        //dataOutput << Score() << ", "<<(CollisionTime-16*Score())/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput.close();
    
        ofstream forageDataOutput((header+"ForageData.txt").c_str(), ios::app);
        if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
        for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
        forageDataOutput<<"\n";
        forageDataOutput.close();
        
      }  

}


argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void CPFA_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	//ofstream log_output_stream;
	//log_output_stream.open("time.txt", ios::app);
	//log_output_stream << t << ", " << GetSpace().GetSimulationClock() << ", " << GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() << endl;
	//log_output_stream.close();
	    for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);
		if(PheromoneList[i].IsActive()) {
			new_p_list.push_back(PheromoneList[i]);
		}
      }
     	PheromoneList = new_p_list;
	new_p_list.clear();
}
void CPFA_loop_functions::SetFoodDistribution() {
	switch(FoodDistribution) {
		case 0:
			RandomFoodDistribution();
			break;
		case 1:
			ClusterFoodDistribution();
			break;
		case 2:
			PowerLawFoodDistribution();
			break;
		default:
			argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
	}
}

void CPFA_loop_functions::RandomFoodDistribution() {
	FoodList.clear();
        FoodColoringList.clear();
	argos::CVector2 placementPosition;

	for(size_t i = 0; i < FoodItemCount; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		FoodList.push_back(placementPosition);
		FoodColoringList.push_back(argos::CColor::BLACK);
	}
}

 
void CPFA_loop_functions::ClusterFoodDistribution() {
        FoodList.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterWidthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	FoodItemCount = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterWidthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < ClusterWidthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}


void CPFA_loop_functions::PowerLawFoodDistribution() {
 FoodList.clear();
    FoodColoringList.clear();
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
		}
	FoodItemCount = foodPlaced;
}
 
bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
	argos::CVector2 placementPosition = p;

	argos::Real foodOffset   = 3.0 * FoodRadius;
	argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
	argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

	argos::Real x_min = p.GetX() - FoodRadius;
	argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

	argos::Real y_min = p.GetY() - FoodRadius;
	argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

	if((x_min < (ForageRangeX.GetMin() + FoodRadius))
			|| (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
			(y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
			(y_max > (ForageRangeY.GetMax() - FoodRadius)))
	{
		return true;
	}

	for(size_t j = 0; j < length; j++) {
		for(size_t k = 0; k < width; k++) {
			if(IsCollidingWithFood(placementPosition)) return true;
			if(IsCollidingWithNest(placementPosition)) return true;
			placementPosition.SetX(placementPosition.GetX() + foodOffset);
		}

		placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
		placementPosition.SetY(placementPosition.GetY() + foodOffset);
	}

	return false;
}

  
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

      return ( (p - NestPosition).SquareLength() < NRPB_squared) ;
}

bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int CPFA_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double CPFA_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double CPFA_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double CPFA_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double CPFA_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double CPFA_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double CPFA_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double CPFA_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real CPFA_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void CPFA_loop_functions::SetTrial(unsigned int v) {
}

void CPFA_loop_functions::setScore(double s) {
	score = s;
    
	if (score >= NumDistributedFood) {
		PostExperiment();
	}
}

double CPFA_loop_functions::Score() {	
	return score;
}

void CPFA_loop_functions::increaseNumDistributedFoodByOne(){
    NumDistributedFood++;
}

void CPFA_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}


void CPFA_loop_functions::ResetAdaptiveRegions() {
    AdaptiveRegions.clear();
    RecentDiscoveryTicks.clear();
    AdaptiveTargetClaims.clear();

    AdaptiveRegion root;
    root.MinX = ForageRangeX.GetMin();
    root.MaxX = ForageRangeX.GetMax();
    root.MinY = ForageRangeY.GetMin();
    root.MaxY = ForageRangeY.GetMax();
    root.Visits = 0;
    root.ResourceHits = 0;
    root.ResourceWeight = 0.0;
    for(size_t i = 0; i < 4; ++i) root.Children[i] = -1;
    AdaptiveRegions.push_back(root);
}

void CPFA_loop_functions::RecordVisitedLocations(const std::vector<argos::CVector2>& c_points) {
    if(AdaptiveRegions.empty()) ResetAdaptiveRegions();
    for(size_t i = 0; i < c_points.size(); ++i) {
        InsertAdaptiveObservation(c_points[i], false, 0);
    }
}

void CPFA_loop_functions::RecordResourceDiscovery(const argos::CVector2& c_point, size_t un_density) {
    if(AdaptiveRegions.empty()) ResetAdaptiveRegions();
    InsertAdaptiveObservation(c_point, true, un_density);

    size_t now = GetSpace().GetSimulationClock();
    RecentDiscoveryTicks.push_back(now);
    while(!RecentDiscoveryTicks.empty() && now - RecentDiscoveryTicks.front() > AdaptiveClusterWindowTicks) {
        RecentDiscoveryTicks.pop_front();
    }
}

void CPFA_loop_functions::InsertAdaptiveObservation(const argos::CVector2& c_point, bool b_resource_hit, size_t un_density) {
    InsertAdaptiveObservation(0, c_point, b_resource_hit, un_density, 0);
}

void CPFA_loop_functions::InsertAdaptiveObservation(size_t un_region,
                                                    const argos::CVector2& c_point,
                                                    bool b_resource_hit,
                                                    size_t un_density,
                                                    size_t un_depth) {
    AdaptiveRegion& region = AdaptiveRegions[un_region];
    region.Visits++;
    if(b_resource_hit) {
        region.ResourceHits++;
        region.ResourceWeight += 1.0 + (Real)un_density;
    }

    if(region.IsLeaf() &&
       un_depth < AdaptiveMaxDepth &&
       region.Visits >= AdaptiveSplitVisitThreshold) {
        SplitAdaptiveRegion(un_region);
    }

    if(!AdaptiveRegions[un_region].IsLeaf()) {
        const AdaptiveRegion& activeRegion = AdaptiveRegions[un_region];
        Real midX = (activeRegion.MinX + activeRegion.MaxX) * 0.5;
        Real midY = (activeRegion.MinY + activeRegion.MaxY) * 0.5;
        int child = (c_point.GetX() >= midX ? 1 : 0) + (c_point.GetY() >= midY ? 2 : 0);
        InsertAdaptiveObservation(activeRegion.Children[child], c_point, b_resource_hit, un_density, un_depth + 1);
    }
}

void CPFA_loop_functions::SplitAdaptiveRegion(size_t un_region) {
    AdaptiveRegion region = AdaptiveRegions[un_region];
    if(!region.IsLeaf()) return;

    Real midX = (region.MinX + region.MaxX) * 0.5;
    Real midY = (region.MinY + region.MaxY) * 0.5;
    int childIndices[4];

    for(size_t i = 0; i < 4; ++i) {
        AdaptiveRegion child;
        child.MinX = (i % 2 == 0) ? region.MinX : midX;
        child.MaxX = (i % 2 == 0) ? midX : region.MaxX;
        child.MinY = (i < 2) ? region.MinY : midY;
        child.MaxY = (i < 2) ? midY : region.MaxY;
        child.Visits = 0;
        child.ResourceHits = 0;
        child.ResourceWeight = 0.0;
        for(size_t j = 0; j < 4; ++j) child.Children[j] = -1;
        childIndices[i] = AdaptiveRegions.size();
        AdaptiveRegions.push_back(child);
    }

    for(size_t i = 0; i < 4; ++i) {
        AdaptiveRegions[un_region].Children[i] = childIndices[i];
    }
}

void CPFA_loop_functions::CollectAdaptiveLeaves(size_t un_region, std::vector<size_t>& vec_leaves) const {
    const AdaptiveRegion& region = AdaptiveRegions[un_region];
    if(region.IsLeaf()) {
        vec_leaves.push_back(un_region);
        return;
    }

    for(size_t i = 0; i < 4; ++i) {
        if(region.Children[i] >= 0) {
            CollectAdaptiveLeaves(region.Children[i], vec_leaves);
        }
    }
}

Real CPFA_loop_functions::ScoreAdaptiveRegion(const AdaptiveRegion& s_region) {
    CVector2 center((s_region.MinX + s_region.MaxX) * 0.5,
                    (s_region.MinY + s_region.MaxY) * 0.5);

    Real pheromoneScore = 0.0;
    for(size_t p = 0; p < PheromoneList.size(); ++p) {
        if(PheromoneList[p].IsActive()) {
            Real d2 = (center - PheromoneList[p].GetLocation()).SquareLength();
            pheromoneScore += PheromoneList[p].GetWeight() *
                              (1.0 + (Real)PheromoneList[p].GetResourceDensity()) /
                              (1.0 + d2);
        }
    }

    Real fidelityScore = 0.0;
    for(map<string, CVector2>::const_iterator f = FidelityList.begin(); f != FidelityList.end(); ++f) {
        Real d2 = (center - f->second).SquareLength();
        fidelityScore += 1.0 / (1.0 + d2);
    }

    Real resourceProbability = s_region.ResourceWeight / (1.0 + (Real)s_region.Visits);
    Real exploration = AdaptiveExploreWeight / (1.0 + (Real)s_region.Visits);
    Real regionWidth = std::max<Real>(0.001, s_region.MaxX - s_region.MinX);
    Real regionHeight = std::max<Real>(0.001, s_region.MaxY - s_region.MinY);
    Real precisionBonus = 0.05 / std::sqrt(regionWidth * regionHeight);

    return exploration +
           AdaptivePheromoneWeight * pheromoneScore +
           AdaptiveResourceWeight * resourceProbability +
           AdaptiveClusterProtectWeight * fidelityScore +
           precisionBonus;
}

size_t CPFA_loop_functions::CountAdaptiveTargetClaims(size_t un_region) const {
    size_t count = 0;
    for(size_t i = 0; i < AdaptiveTargetClaims.size(); ++i) {
        if(AdaptiveTargetClaims[i].second == un_region) {
            count++;
        }
    }
    return count;
}

CVector2 CPFA_loop_functions::SampleAdaptiveRegion(const AdaptiveRegion& s_region) {
    Real margin = std::min<Real>(0.05, std::min(s_region.MaxX - s_region.MinX,
                                               s_region.MaxY - s_region.MinY) * 0.25);
    Real x = RNG->Uniform(CRange<Real>(s_region.MinX + margin, s_region.MaxX - margin));
    Real y = RNG->Uniform(CRange<Real>(s_region.MinY + margin, s_region.MaxY - margin));
    return CVector2(x, y);
}

bool CPFA_loop_functions::SelectAdaptiveSearchTarget(argos::CVector2& c_target) {
    if(AdaptiveRegions.empty()) ResetAdaptiveRegions();
    size_t now = GetSpace().GetSimulationClock();
    while(!AdaptiveTargetClaims.empty() && now - AdaptiveTargetClaims.front().first > AdaptiveClaimWindowTicks) {
        AdaptiveTargetClaims.pop_front();
    }

    struct Candidate { Real Score; size_t Region; };
    std::vector<size_t> leaves;
    std::vector<Candidate> candidates;
    CollectAdaptiveLeaves(0, leaves);

    for(size_t i = 0; i < leaves.size(); ++i) {
        Real balancingPenalty = 1.0 + (Real)CountAdaptiveTargetClaims(leaves[i]);
        Candidate c = {ScoreAdaptiveRegion(AdaptiveRegions[leaves[i]]) / balancingPenalty, leaves[i]};
        candidates.push_back(c);
    }

    if(candidates.empty()) return false;
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b){ return a.Score > b.Score; });

    size_t keep = std::min<size_t>(AdaptiveTopCandidates, candidates.size());
    Real total = 0.0;
    for(size_t i = 0; i < keep; ++i) total += candidates[i].Score + AdaptiveRandomWeight;
    if(total <= 0.0) return false;

    Real pick = RNG->Uniform(CRange<Real>(0.0, total));
    size_t selected = 0;
    for(size_t i = 0; i < keep; ++i) {
        pick -= candidates[i].Score + AdaptiveRandomWeight;
        if(pick <= 0.0) {
            selected = i;
            break;
        }
    }

    c_target = SampleAdaptiveRegion(AdaptiveRegions[candidates[selected].Region]);
    AdaptiveTargetClaims.push_back(std::make_pair(now, candidates[selected].Region));
    return true;
}

bool CPFA_loop_functions::IsClusteredResourceMode() {
    if(AdaptiveClusterWindowTicks == 0) return false;
    size_t now = GetSpace().GetSimulationClock();
    while(!RecentDiscoveryTicks.empty() && now - RecentDiscoveryTicks.front() > AdaptiveClusterWindowTicks) {
        RecentDiscoveryTicks.pop_front();
    }
    return RecentDiscoveryTicks.size() >= AdaptiveClusterHitThreshold;
}

REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
