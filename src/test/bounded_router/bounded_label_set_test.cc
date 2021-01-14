#include "gtest/gtest.h"

#include "log.hh"

#include "bounded_router/bounded_label_set.hh"

TEST(BoundedLabelSet, Insertion)
{
  BoundedLabelSet labels;

  for(idx i = 0; i < 10; ++i)
  {
    Units::Mass fuel((10 - i) * Units::SI::kilogram);

    auto label = std::make_shared<BoundedLabel>((double) i,
                                                fuel,
                                                std::make_shared<BoundedLabel>(Vertex()),
                                                Edge());

    labels.insert(label);

    EXPECT_EQ(labels.size(), i + 1);
  }
}

TEST(BoundedLabelSet, DominatedInsertion)
{
  BoundedLabelSet labels;

  auto first = std::make_shared<BoundedLabel>(1,
                                              Units::Mass(1 * Units::SI::kilogram),
                                              std::make_shared<BoundedLabel>(Vertex()),
                                              Edge());

  auto second = std::make_shared<BoundedLabel>(2,
                                               Units::Mass(2 * Units::SI::kilogram),
                                               std::make_shared<BoundedLabel>(Vertex()),
                                               Edge());

  labels.insert(first);
  labels.insert(second);

  EXPECT_EQ(labels.size(), 1);

  EXPECT_EQ(*labels.get_labels().begin(), first);
}

TEST(BoundedLabelSet, SimpleDominatingInsertion)
{
  BoundedLabelSet labels;

  Units::Mass fuel(0 * Units::SI::kilogram);

  auto dominated = std::make_shared<BoundedLabel>(1,
                                                  fuel,
                                                  std::make_shared<BoundedLabel>(Vertex()),
                                                  Edge());

  labels.insert(dominated);

  auto dominating = std::make_shared<BoundedLabel>(0,
                                                   fuel,
                                                   std::make_shared<BoundedLabel>(Vertex()),
                                                   Edge());

  labels.insert(dominating);

  EXPECT_EQ(labels.size(), 1);

  EXPECT_EQ(*labels.get_labels().begin(), dominating);
}

TEST(BoundedLabelSet, DuplicateInsertion)
{
  BoundedLabelSet labels;

  Units::Mass fuel(0 * Units::SI::kilogram);

  auto first = std::make_shared<BoundedLabel>(1,
                                              0 * Units::SI::kilogram,
                                              std::make_shared<BoundedLabel>(Vertex()),
                                              Edge());

  labels.insert(first);

  auto second = std::make_shared<BoundedLabel>(0,
                                               1 * Units::SI::kilogram,
                                               std::make_shared<BoundedLabel>(Vertex()),
                                               Edge());

  labels.insert(second);

  EXPECT_EQ(labels.size(), 2);

  EXPECT_FALSE(labels.insert(first));
  EXPECT_FALSE(labels.insert(second));
}

TEST(BoundedLabelSet, DominatingInsertion)
{
  BoundedLabelSet labels;

  for(idx i = 0; i < 10; ++i)
  {
    Units::Mass fuel((10 - i) * Units::SI::kilogram);

    auto label = std::make_shared<BoundedLabel>((double) i,
                                                fuel,
                                                std::make_shared<BoundedLabel>(Vertex()),
                                                Edge());

    labels.insert(label);
  }

  auto dominating = std::make_shared<BoundedLabel>(0,
                                                   Units::Mass(0 * Units::SI::kilogram),
                                                   std::make_shared<BoundedLabel>(Vertex()),
                                                   Edge());

  labels.insert(dominating);

  EXPECT_EQ(labels.size(), 1);
}

TEST(BoundedLabelSet, PartiallyDominatingInsertion)
{
  BoundedLabelSet labels;

  for(idx i = 0; i < 10; ++i)
  {
    Units::Mass fuel((10 - i) * Units::SI::kilogram);

    auto label = std::make_shared<BoundedLabel>((double) i,
                                                fuel,
                                                std::make_shared<BoundedLabel>(Vertex()),
                                                Edge());

    labels.insert(label);
  }

  auto dominating = std::make_shared<BoundedLabel>(5,
                                                   Units::Mass(0 * Units::SI::kilogram),
                                                   std::make_shared<BoundedLabel>(Vertex()),
                                                   Edge());

  labels.insert(dominating);

  EXPECT_EQ(labels.size(), 6);
}

TEST(BoundedLabelSet, UndominatedInsertion)
{
  BoundedLabelSet labels;

  for(idx i = 0; i < 10; i += 2)
  {
    Units::Mass fuel((10 - i) * Units::SI::kilogram);

    auto label = std::make_shared<BoundedLabel>((double) i,
                                                fuel,
                                                std::make_shared<BoundedLabel>(Vertex()),
                                                Edge());

    labels.insert(label);
  }

  EXPECT_EQ(labels.size(), 5);

  idx size = 5;

  for(idx i = 1; i < 10; i += 2)
  {
    Units::Mass fuel((10 - i) * Units::SI::kilogram);

    auto label = std::make_shared<BoundedLabel>((double) i,
                                                fuel,
                                                std::make_shared<BoundedLabel>(Vertex()),
                                                Edge());

    labels.insert(label);

    EXPECT_EQ(labels.size(), ++size);
  }
}
