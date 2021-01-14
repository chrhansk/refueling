#include <gtest/gtest.h>

#include "pareto_front.hh"

class ParetoFixture : public ::testing::Test
{
};


TEST_F(ParetoFixture, construct)
{
  ParetoFront<int, int> pareto_front(std::make_pair(0, 10),
                                     std::make_pair(10, 0));

  EXPECT_EQ(pareto_front.size(), 2);
}

TEST_F(ParetoFixture, intervals)
{
  typedef ParetoFront<int, int>::Entry Entry;

  const Entry left = std::make_pair(0, 10);
  const Entry right = std::make_pair(10, 0);

  ParetoFront<int, int> pareto_front(left, right);

  typedef ParetoFront<int, int>::Interval Interval;

  std::vector<Interval> intervals;

  for(const auto& interval : pareto_front.intervals())
  {
    intervals.push_back(interval);
  }

  Interval interval = intervals.front();

  EXPECT_EQ(intervals.size(), 1);

  EXPECT_EQ(interval.get_begin(), left);
  EXPECT_EQ(interval.get_end(), right);
}

TEST_F(ParetoFixture, insert_simple)
{
  typedef ParetoFront<int, int>::Entry Entry;

  const Entry left = std::make_pair(0, 10);
  const Entry right = std::make_pair(10, 0);

  ParetoFront<int, int> pareto_front(left, right);

  bool inserted = pareto_front.insert(Entry(5, 5));

  EXPECT_TRUE(inserted);

  EXPECT_EQ(pareto_front.size(), 3);
}

TEST_F(ParetoFixture, insert_dominated)
{
  typedef ParetoFront<int, int>::Entry Entry;

  const Entry left = std::make_pair(0, 10);
  const Entry right = std::make_pair(10, 0);

  ParetoFront<int, int> pareto_front(left, right);

  bool inserted = pareto_front.insert(Entry(5, 15));

  EXPECT_FALSE(inserted);

  EXPECT_EQ(pareto_front.size(), 2);
}

TEST_F(ParetoFixture, insert_dominating)
{
  typedef ParetoFront<int, int>::Entry Entry;

  const Entry left = std::make_pair(0, 10);
  const Entry right = std::make_pair(10, 0);

  const Entry dominating = std::make_pair(4, 4);

  ParetoFront<int, int> pareto_front(left, right);

  {
    bool inserted = pareto_front.insert(Entry(5, 5));

    EXPECT_TRUE(inserted);
  }

  {
    bool inserted = pareto_front.insert(dominating);

    EXPECT_TRUE(inserted);
  }

  auto interval = *(pareto_front.intervals().begin());

  EXPECT_EQ(interval.get_end(), dominating);

  EXPECT_EQ(pareto_front.size(), 3);
}
