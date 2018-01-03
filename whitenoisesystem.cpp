#include "whitenoisesystem.h"

#include <boost/random/normal_distribution.hpp>
#include <boost/random/lagged_fibonacci.hpp>

WhiteNoiseSystem::WhiteNoiseSystem(
  const boost::numeric::ublas::matrix<double>& real_value,
  const boost::numeric::ublas::matrix<double>& stddev)
  : m_mean(real_value),
    m_stddev(stddev)
{
  assert(m_mean.size1() == m_stddev.size1() && "Every value must have one measurement noise");
  assert(m_mean.size2() == m_stddev.size2() && "Every value must have one measurement noise");
}

double WhiteNoiseSystem::GetRandomNormal(const double mean, const double sigma)
{
  boost::normal_distribution<double> norm_dist(mean, sigma);
  static boost::lagged_fibonacci19937 engine;
  const double value = norm_dist.operator () <boost::lagged_fibonacci19937>((engine));
  return value;
}

const boost::numeric::ublas::matrix<double> WhiteNoiseSystem::Measure() const
{
  const std::size_t n_rows = m_mean.size1();
  const std::size_t n_cols = m_mean.size2();
  boost::numeric::ublas::matrix<double> measured(n_rows,n_cols);
  for (std::size_t row = 0; row!=n_rows; ++row)
  {
    for (std::size_t col = 0; col!=n_cols; ++col)
    {
      measured(row,col) = GetRandomNormal(m_mean(row,col),m_stddev(row,col));
    }
  }
  return measured;
}
