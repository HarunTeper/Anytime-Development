#ifndef ANYTIME_TEMPLATE_HPP
#define ANYTIME_TEMPLATE_HPP

template <typename InputType, typename ReturnType>
class Anytime {
 public:
  virtual ~Anytime() = default;

  // Pure virtual function for blocking operation
  virtual ReturnType blockingFunction(const InputType& input) = 0;

  // Pure virtual function for non-blocking operation
  virtual void nonBlockingFunction(const InputType& input,
                                   std::shared_ptr<ReturnType> result) = 0;
};

#endif  // ANYTIME_TEMPLATE_HPP