#ifndef INDEX_MAP_HH
#define INDEX_MAP_HH

#include "util.hh"

template<class K, class V>
class Func
{
public:
  virtual V operator()(const K& key) const = 0;
};

template<class K, class V>
class IndexMap;

template<class K, class V>
class IndexValueMap : public Func<K, V>
{
private:
  const IndexMap<K, V> *map;

public:
  IndexValueMap(const IndexMap<K, V>& map)
    : map(&map)
  {}

  V operator()(const K& key) const override
  {
    return V((*map)(key));
  }

  ~IndexValueMap() {}
};

template<class K, class V>
class IndexMap : public Func<K, const V&>
{
private:
  std::vector<V> values;
  IndexValueMap<K, V> valueMap;

public:
  IndexMap(idx size, V value = V())
    : values(size, value),
      valueMap(*this)
  {}

  IndexMap(const IndexMap& other)
    : values(other.values),
      valueMap(*this)
  {}

  IndexMap(IndexMap&& other)
    : values(std::move(other.values)),
      valueMap(*this)
  {}

  IndexMap(idx size, V* vals)
  {
    values.reserve(size);

    for(idx i = 0; i < size; ++i)
    {
      values.push_back(vals[i]);
    }
  }

  const V& operator()(const K& key) const
  {
    return values.at(key.get_index());
  }

  V& operator()(const K& key)
  {
    return values.at(key.get_index());
  }

  IndexMap& operator=(const IndexMap& other)
  {
    values = other.values;
    valueMap = IndexValueMap<K, V>(*this);

    return *this;
  }

  void set_value(const K& key, const V& value)
  {
    values[key.get_index()] = value;
  }

  void reset(const V& value)
  {
    std::fill(values.begin(), values.end(), value);
  }

  void extend(const K& key, const V& value)
  {
    assert(key.get_index() == values.size());

    values.push_back(value);
  }

  operator const IndexValueMap<K, V>&() const
  {
    return valueMap;
  }

  const IndexValueMap<K, V>& get_values() const
  {
    return valueMap;
  }
};


#endif /* INDEX_MAP_HH */
