#ifndef FILTER_BASE_HPP_
#define FILTER_BASE_HPP_

namespace filters {

/**
 *
 */
template <typename T, typename dataT>
class FilterBase {
public:
  virtual T update(dataT sample) = 0;
};

} /* namespace */

#endif /* FILTER_BASE_HPP_ */
