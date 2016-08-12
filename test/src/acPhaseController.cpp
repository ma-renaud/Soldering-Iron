#include "CppUTest/TestHarness.h"


class AcPhaseController
{
	static constexpr float FREQUENCY = 60.0;

	public:
		AcPhaseController(){}

		float getFrequency()
		{
			return FREQUENCY;
		}
};

TEST_GROUP(AcPhaseControllerGroup)
{
	void setup()
	{
		// Init stuff
	}

	void teardown()
	{
		// Uninit stuff
	}
};

TEST(AcPhaseControllerGroup, GetFrequency)
{
	double frequency = 60.0;
	AcPhaseController controller;

	DOUBLES_EQUAL(frequency, controller.getFrequency(), 0.001);
}


