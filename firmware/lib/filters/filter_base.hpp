#ifndef FILTER_BASE_HPP_
#define FILTER_BASE_HPP_

namespace filters {

/**
 *
 */
template <typename T>
class FilterBase {
public:
  virtual T update(T sample) = 0;
};

} /* namespace */

#endif /* FILTER_BASE_HPP_ */
