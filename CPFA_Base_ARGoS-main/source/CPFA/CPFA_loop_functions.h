#ifndef CPFA_LOOP_FUNCTIONS_H
#define CPFA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/CPFA/CPFA_controller.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <deque>
#include <map>
#include <utility>

using namespace argos;
using namespace std;

static const size_t GENOME_SIZE = 7; // There are 7 parameters to evolve

class CPFA_loop_functions : public argos::CLoopFunctions
{

	friend class CPFA_controller;
	friend class CPFA_qt_user_functions;

	public:

		CPFA_loop_functions();
	   
		void Init(argos::TConfigurationNode &t_tree);
		void Reset();
		void PreStep();
		void PostStep();
		bool IsExperimentFinished();
		void PostExperiment();
		argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

		// GA Functions
		
		/* Configures the robot controller from the genome */
		void ConfigureFromGenome(Real* pf_genome);
		/* Calculates the performance of the robot in a trial */
		Real Score();
	
		/**
		 * Returns the current trial.
		 */
		UInt32 GetTrial() const;
	
		/**
		 * Sets the current trial.
		 * @param un_trial The trial number.
		 */
		void SetTrial(UInt32 un_trial);
	
		/* public helper functions */
		void UpdatePheromoneList();
		void SetFoodDistribution();

			void RecordVisitedLocations(const std::vector<argos::CVector2>& c_points);
			void RecordResourceDiscovery(const argos::CVector2& c_point, size_t un_density);
			bool SelectAdaptiveSearchTarget(argos::CVector2& c_target);
			bool IsClusteredResourceMode();

		argos::Real getSimTimeInSeconds();

		std::vector<argos::CColor>   TargetRayColorList;

		unsigned int getNumberOfRobots();
        void increaseNumDistributedFoodByOne();
		double getProbabilityOfSwitchingToSearching();
		double getProbabilityOfReturningToNest();
		double getUninformedSearchVariation();
		double getRateOfInformedSearchDecay();
		double getRateOfSiteFidelity();
		double getRateOfLayingPheromone();
		double getRateOfPheromoneDecay();

	protected:

		void setScore(double s);

		argos::CRandom::CRNG* RNG;
                size_t NumDistributedFood;
		size_t MaxSimTime;
		size_t ResourceDensityDelay;
		size_t RandomSeed;
		size_t SimCounter;
		size_t MaxSimCounter;
		size_t VariableFoodPlacement;
		size_t OutputData;
		size_t DrawDensityRate;
		size_t DrawIDs;
		size_t DrawTrails;
		size_t DrawTargetRays;
		size_t UseAHCFA;
		size_t FoodDistribution;
		size_t FoodItemCount;
		size_t PowerlawFoodUnitCount;
		size_t NumberOfClusters;
		size_t ClusterWidthX;
		size_t ClusterWidthY;
		size_t PowerRank;
                size_t ArenaWidth;
                size_t SimTime; 
                Real curr_time_in_minutes; 
                Real last_time_in_minutes; 
  
		/* CPFA variables */
		argos::Real ProbabilityOfSwitchingToSearching;
		argos::Real ProbabilityOfReturningToNest;
		argos::CRadians UninformedSearchVariation;
		argos::Real RateOfInformedSearchDecay;
		argos::Real RateOfSiteFidelity;
		argos::Real RateOfLayingPheromone;
		argos::Real RateOfPheromoneDecay;

		/* physical robot & world variables */
		argos::Real FoodRadius;
		argos::Real FoodRadiusSquared;
		argos::Real NestRadius;
		argos::Real NestRadiusSquared;
		argos::Real NestElevation;
		argos::Real SearchRadiusSquared;

		/* list variables for food & pheromones */
		std::vector<argos::CVector2> FoodList;
		std::vector<argos::CColor>   FoodColoringList;
		vector<argos::CVector2> CollectedFoodList;
                map<string, argos::CVector2> FidelityList; 
		std::vector<Pheromone>   PheromoneList; 
		std::vector<argos::CRay3>    TargetRayList;
		argos::CRange<argos::Real>   ForageRangeX;
		argos::CRange<argos::Real>   ForageRangeY;
  
                Real   CollisionTime;
                size_t currCollisionTime; 
                size_t lastCollisionTime; 
                size_t lastNumCollectedFood;
                size_t currNumCollectedFood;
                size_t Num_robots;
      
                vector<size_t>		ForageList;
		argos::CVector2 NestPosition;

			struct AdaptiveRegion {
				argos::Real MinX;
				argos::Real MaxX;
				argos::Real MinY;
				argos::Real MaxY;
				size_t Visits;
				size_t ResourceHits;
				argos::Real ResourceWeight;
				int Children[4];

				bool IsLeaf() const {
					return Children[0] < 0;
				}
			};

			std::vector<AdaptiveRegion> AdaptiveRegions;
			std::deque<size_t> RecentDiscoveryTicks;
			std::deque<std::pair<size_t, size_t> > AdaptiveTargetClaims;
			size_t AdaptiveMaxDepth;
			size_t AdaptiveSplitVisitThreshold;
			size_t AdaptiveClusterWindowTicks;
			size_t AdaptiveClusterHitThreshold;
			size_t AdaptiveClaimWindowTicks;
			size_t AdaptiveTopCandidates;
			argos::Real AdaptiveExploreWeight;
			argos::Real AdaptivePheromoneWeight;
			argos::Real AdaptiveResourceWeight;
			argos::Real AdaptiveClusterProtectWeight;
			argos::Real AdaptiveRandomWeight;

	private:

		/* private helper functions */
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();
		void ResetAdaptiveRegions();
		void InsertAdaptiveObservation(const argos::CVector2& c_point, bool b_resource_hit, size_t un_density);
		void InsertAdaptiveObservation(size_t un_region, const argos::CVector2& c_point, bool b_resource_hit, size_t un_density, size_t un_depth);
		void SplitAdaptiveRegion(size_t un_region);
		void CollectAdaptiveLeaves(size_t un_region, std::vector<size_t>& vec_leaves) const;
		argos::Real ScoreAdaptiveRegion(const AdaptiveRegion& s_region);
		size_t CountAdaptiveTargetClaims(size_t un_region) const;
		argos::CVector2 SampleAdaptiveRegion(const AdaptiveRegion& s_region);
                bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		double score;
		int PrintFinalScore;
};

#endif /* CPFA_LOOP_FUNCTIONS_H */
