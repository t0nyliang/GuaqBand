#ifndef MATOPTIMIZATION_EIGEN_UTILS_H
#define MATOPTIMIZATION_EIGEN_UTILS_H

#include <Eigen/Dense>

template <typename Scalar, size_t N>
void vector_to_eigen(const std::vector<std::array<Scalar, N>> &from,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &to)
{
    if (from.size() == 0) {
        to.resize(0, N);
        return;
    }
    to.resize(from.size(), N);
    for (size_t i = 0; i < from.size(); ++i) {
        for (size_t j = 0; j < N; ++j) {
            to(i, j) = from[i][j];
        }
    }
}

template <typename Scalar>
void vector_to_eigen(const std::vector<std::pair<Scalar, Scalar>> &from,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &to)
{
    if (from.size() == 0) {
        to.resize(0, 2);
        return;
    }
    to.resize(from.size(), 2);
    for (size_t i = 0; i < from.size(); ++i) {
        to.row(i) << from[i].first, from[i].second;
    }
}

template <typename Scalar, int N>
void vector_to_eigen(const std::vector<Eigen::Matrix<Scalar, N, 1>> &from,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &to)
{
    if (from.size() == 0) {
        to.resize(0, N);
        return;
    }
    to.resize(from.size(), N);
    for (size_t i = 0; i < from.size(); ++i) {
        for (size_t j = 0; j < N; ++j) {
            to(i, j) = from[i][j];
        }
    }
}

template <typename Scalar>
void vector_to_eigen(const std::vector<Scalar> &from,
                     Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &to)
{
    if (from.size() == 0) {
        to.resize(0);
        return;
    }
    to.resize(from.size(), 1);
    for (size_t i = 0; i < from.size(); ++i) {
        to(i) = from[i];
    }
}

template <typename Scalar, size_t N>
void eigen_to_vector(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &from,
                     std::vector<std::array<Scalar, N>> &to)
{
    if (from.cols() != N) {
        throw std::invalid_argument("Wrong number of columns");
    }
    to.resize(from.rows());
    for (size_t i = 0; i < to.size(); ++i) {
        for (size_t j = 0; j < N; ++j) {
            to[i][j] = from(i, j);
        }
    }
}

template <typename Scalar>
void eigen_to_vector(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &from,
                     std::vector<std::pair<Scalar, Scalar>> &to)
{
    if (from.cols() != 2) {
        throw std::invalid_argument("Wrong number of columns");
    }
    to.resize(from.rows());
    for (size_t i = 0; i < to.size(); ++i) {
        to[i] = std::make_pair(from[i][0], from[i][1]);
    }
}

#endif //MATOPTIMIZATION_EIGEN_UTILS_H
