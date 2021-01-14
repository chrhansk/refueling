#include "bounded_label_set.hh"

#include <cmath>

bool BoundedLabelSet::insert(std::shared_ptr<BoundedLabel> label)
{
  assert(label);

  Units::Mass fuel = label->get_fuel();

  if(labels.empty())
  {
    labels.insert(std::make_pair(fuel, label));

    return true;
  }

  if(is_dominated(label))
  {
    return false;
  }

  auto upper_bound = labels.lower_bound(fuel);

  while(upper_bound != std::end(labels))
  {
    std::shared_ptr<BoundedLabel> bound_label = upper_bound->second;

    if(label->dominates(*(bound_label)))
    {
      upper_bound = labels.erase(upper_bound);
    }
    else
    {
      break;
    }
  }

  // if no rounding schema is applied
  // it has to hold that the insertion is
  // successful...

  labels.insert(std::make_pair(fuel, label));


  return true;
}

bool BoundedLabelSet::is_dominated(std::shared_ptr<BoundedLabel> label)
{
  assert(label);

  Units::Mass fuel = label->get_fuel();

  auto lower_bound = labels.lower_bound(fuel);

  if(lower_bound == std::end(labels))
  {
    --lower_bound;
  }

  assert(lower_bound != std::end(labels));

  while(lower_bound->first > fuel && lower_bound != std::begin(labels))
  {
    --lower_bound;
  }

  std::shared_ptr<BoundedLabel> bound_label = lower_bound->second;

  if(bound_label->dominates(*label))
  {
    return true;
  }

  return false;
}
