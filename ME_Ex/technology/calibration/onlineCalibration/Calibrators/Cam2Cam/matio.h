/**
 * \file matio.h
 * \brief Routines for reading and writing hdf data into our matrix class.
 *
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include "technology/mobilib/float/common/MEmath/mat.h"


namespace CalibUtils
{
template <int ROWS, int COLS, class T>
bool loadHdf5(std::string const &filename, std::string const& dataset, std::vector < Float::MEmath::Mat<ROWS, COLS, T> > & out)
{
    arma::cube data;
    bool ok = data.load(arma::hdf5_name(filename, dataset));
    if (!ok || data.n_rows != ROWS || data.n_cols != COLS)
    {
        return false;
    }
    out.resize(data.n_slices);
    arma::mat slice(data.n_rows, data.n_cols);
    for (unsigned i = 0; i < data.n_slices; i++)
    {
        slice = data.slice(i).st();
        std::memcpy(out[i].begin(), slice.begin(), sizeof(T)*ROWS*COLS);
    }
    return true;
}

template <int ROWS, int COLS, class T>
bool saveHdf5(std::string const &filename, std::string const &dataset, std::vector<Float::MEmath::Mat<ROWS, COLS, T>> const &data, std::string const& mode = "w")
{
    arma::cube armaData(ROWS, COLS, data.size());
    arma::mat slice(COLS, ROWS);
    for (unsigned i = 0; i < data.size(); i++)
    {
        std::memcpy(slice.begin(), data[i].begin(), sizeof(T) * ROWS * COLS);
        armaData.slice(i) = slice.st();
    }

    if (mode == "w")
    {
        return armaData.save(arma::hdf5_name(filename, dataset));
    }
    else if (mode == "u")
    {
        return armaData.save(arma::hdf5_name(filename, dataset, arma::hdf5_opts::append));
    }
    else
    {
        throw std::runtime_error("saveHdf5::Unrecognized mode string.");
        return false;
    }
}
}