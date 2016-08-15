#include "CppUTest/TestHarness.h"

class AcPhaseController
{
	public:
		AcPhaseController(float frequency, uint32_t clockSpeed, uint32_t prescaler)
		{
			m_frequency = frequency;
			m_clockSpeed = clockSpeed;
			m_prescaler = prescaler;

			m_delay = 0;
			m_duration = 0;
		}

		float getFrequency()
		{
			return m_frequency;
		}

		uint32_t getPulse()
		{
			return m_delay*(m_clockSpeed/(m_prescaler+1));
		}

		uint32_t getPeriod()
		{
			return (m_duration*(m_clockSpeed/(m_prescaler+1)))+getPulse();
		}

		void setDelay(float delay)
		{
			m_delay = delay;
		}

		void setDuration(float duration)
		{
			m_duration = duration;
		}

	private:
		float m_frequency;
		uint32_t m_clockSpeed;
		uint32_t m_prescaler;
		float m_delay;
		float m_duration;
};

TEST_GROUP(AcPhaseControllerGroup)
{
	float frequency = 60.0;
	uint32_t clockSpeed = 48000000;
	uint32_t prescaler = 23;

	AcPhaseController *pController;

	void setup()
	{
		pController = new AcPhaseController(frequency,clockSpeed,prescaler);
	}

	void teardown()
	{
		delete pController;
	}
};

TEST(AcPhaseControllerGroup, GetFrequency)
{
	double frequency = 60.0;

	DOUBLES_EQUAL(frequency, pController->getFrequency(), 0.001);
}

TEST(AcPhaseControllerGroup, GetPulse)
{
	uint32_t pulse = 8000;
	float delayInSecond = 0.004;

	pController->setDelay(delayInSecond);

	LONGS_EQUAL(pulse, pController->getPulse());
}

TEST(AcPhaseControllerGroup, GetDuration)
{
	uint32_t period = 8400;
	float delayInSecond = 0.004;
	float durationInSecond = 0.0002;

	pController->setDelay(delayInSecond);
	pController->setDuration(durationInSecond);

	LONGS_EQUAL(period, pController->getPeriod());
}


