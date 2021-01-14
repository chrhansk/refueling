#ifndef PARETO_FRONT_HH
#define PARETO_FRONT_HH

#include <map>

template<class K,
         class V,
         class Comp = std::less<V>>
class ParetoFront
{
private:
  typedef typename std::map<K, V, std::less<K>> EntryMap;

  EntryMap entries;
  Comp comparator;

  typedef typename EntryMap::const_iterator EntryIterator;

public:

  typedef typename EntryMap::value_type Entry;

  class Interval
  {
  private:
    const Entry& begin;
    const Entry& end;

  public:
    Interval(const Entry& begin,
             const Entry& end)
      : begin(begin),
        end(end)
    {}

    const Entry& get_begin() const
    {
      return begin;
    }

    const Entry& get_end() const
    {
      return end;
    }

  };

  class IntervalIterator
  {
  private:
    EntryIterator first, second;
  public:
    IntervalIterator(const EntryIterator& first,
                     const EntryIterator& second)
      : first(first),
        second(second)
    {}

    IntervalIterator()
    {}

    Interval operator*() const
    {
      return Interval(*first, *second);
    }

    IntervalIterator& operator++()
    {
      ++first;
      ++second;
      return *this;
    }

    bool operator!=(const IntervalIterator& other) const
    {
      return second != other.second;
    }

  };

  class Intervals
  {
    IntervalIterator begin_iterator, end_iterator;

  public:
    Intervals(const EntryMap& entries)
    {
      assert(entries.size() >= 2);

      auto it = std::begin(entries);
      auto next = it;
      ++next;

      begin_iterator = IntervalIterator(it, next);

      auto end = std::end(entries);

      end_iterator = IntervalIterator(it, end);
    }

    IntervalIterator begin()
    {
      return begin_iterator;
    }

    IntervalIterator end()
    {
      return end_iterator;
    }
  };

  class Entries
  {
  private:
    EntryIterator begin_iterator, end_iterator;

  public:
    Entries(const EntryMap& entries)
    {
      assert(entries.size() >= 2);

      begin_iterator = std::begin(entries);
      end_iterator = std::end(entries);
    }

    EntryIterator begin()
    {
      return begin_iterator;
    }

    EntryIterator end()
    {
      return end_iterator;
    }

  };

public:
  ParetoFront(const Entry& left,
              const Entry& right,
              const Comp& comparator = Comp());

  unsigned int size() const
  {
    return entries.size();
  }

  Intervals intervals() const
  {
    return Intervals(entries);
  }

  Entries get_entries() const
  {
    return Entries(entries);
  }

  bool insert(const Entry& entry);

  EntryIterator lower_bound(const K& key) const;

private:
  bool dominates(const Entry& first, const Entry& second) const;

};

template<class K, class V, class Comp>
ParetoFront<K, V, Comp>::ParetoFront(const ParetoFront<K, V, Comp>::Entry& left,
                                     const ParetoFront<K, V, Comp>::Entry& right,
                                     const Comp& comparator)
  : comparator(comparator)
{
  assert(!dominates(left, right));

  entries.insert(left);
  entries.insert(right);
}

template<class K, class V, class Comp>
typename ParetoFront<K, V, Comp>::EntryIterator
ParetoFront<K, V, Comp>::lower_bound(const K& key) const
{
  auto lower_bound = entries.lower_bound(key);

  if(lower_bound == std::end(entries))
  {
    --lower_bound;
  }

  while(lower_bound->first > key && lower_bound != std::begin(entries))
  {
    --lower_bound;
  }

  assert(lower_bound != std::end(entries));

  if(lower_bound == std::begin(entries) && lower_bound->first > key)
  {
    return get_entries().end();
  }

  return EntryIterator(lower_bound);
}

template<class K, class V, class Comp>
bool ParetoFront<K, V, Comp>::dominates(const ParetoFront<K, V, Comp>::Entry& first,
                                        const ParetoFront<K, V, Comp>::Entry& second) const
{
  return first.first <= second.first && !comparator(second.second, first.second);
}

template<class K, class V, class Comp>
bool ParetoFront<K, V, Comp>::insert(const Entry& entry)
{
  auto lower_bound = entries.lower_bound(entry.first);

  if(lower_bound == std::end(entries))
  {
    --lower_bound;
  }

  assert(lower_bound != std::end(entries));

  while(lower_bound->first > entry.first && lower_bound != std::begin(entries))
  {
    --lower_bound;
  }

  if(dominates(*lower_bound, entry))
  {
    return false;
  }

  auto upper_bound = entries.lower_bound(entry.first);

  while(upper_bound != std::end(entries))
  {
    if(dominates(entry, *upper_bound))
    {
      upper_bound = entries.erase(upper_bound);
    }
    else
    {
      break;
    }
  }

  entries.insert(entry);

  return true;
}


#endif /* PARETO_FRONT_HH */
