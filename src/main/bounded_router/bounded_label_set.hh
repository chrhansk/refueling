#ifndef BOUNDED_LABEL_SET_HH
#define BOUNDED_LABEL_SET_HH

#include <map>

#include "units.hh"

#include "bounded_label.hh"

class BoundedLabelSet
{
public:

  class BoundedLabels
  {
  public:
    typedef std::map<Units::Mass, BoundedLabelPtr>::const_iterator SetIterator;

    struct Iterator
    {
      SetIterator iter;

      BoundedLabelPtr operator*() const
      {
        return iter->second;
      }

      const Iterator& operator++()
      {
        ++iter;
        return *this;
      }

      const bool operator!=(const Iterator& other) const
      {
        return iter != other.iter;
      }
    };

    SetIterator begin_it, end_it;

    BoundedLabels(SetIterator begin_it, SetIterator end_it)
      : begin_it(begin_it),
        end_it(end_it)
    {}

    Iterator begin()
    {
      return Iterator{begin_it};
    }

    Iterator end()
    {
      return Iterator{end_it};
    }
  };

private:
  std::map<Units::Mass, BoundedLabelPtr> labels;


public:
  BoundedLabelSet()
  {}

  bool insert(BoundedLabelPtr label);

  bool is_dominated(BoundedLabelPtr label);

  idx size() const
  {
    return labels.size();
  }

  BoundedLabels get_labels() const
  {
    return BoundedLabels(std::begin(labels), std::end(labels));
  }
};


#endif /* BOUNDED_LABEL_SET_HH */
