#pragma once
#include <cstdio>
#include <cassert>
#include <fstream>
#include <vector>
#include <string>
#include <exception>
#include "technology/mobilib/float/common/MEmath/mat.h"

namespace CalibUtils
{
template <size_t ROWS, size_t COLS, class T>
class MatArray
{
  public:
    std::vector<Float::MEmath::Mat<(int)ROWS, (int)COLS, T>> data;
    size_t cols() { return COLS; }
    size_t rows() { return ROWS; }
    size_t slices() { return data.size(); }
    /**
    * Save data to a file.
    * \param filename
    * \return true if succesful
    */
    bool save(std::string const &filename)
    {
        std::FILE *pFile = std::fopen(filename.c_str(), "wb");
        if (pFile == nullptr)
        {
            return false;
        }
        // Write header. Format is rows cols slices, all unsigned ints
        unsigned int size[3] = {ROWS, COLS, (unsigned int)this->slices()};
        for (unsigned int i = 0; i < 3; i++)
        {            
          std::fwrite(&size[i], sizeof size[i], 1, pFile);
        }
        std::fwrite(&data[0], sizeof data[0], this->slices(), pFile);
        std::fclose(pFile);
        return true;
    }
    /**
    * Append data to a file.
    * \param filename
    * \return true if succesful
    */
    bool append(std::string const &filename)
    {
        std::FILE *pFile = std::fopen(filename.c_str(), "rb+");
        if (pFile == nullptr)
        {
            return save(filename);
        }
        // Read and update header. Format is rows cols slices, all unsigned int
        std::vector<unsigned int> size(3);
        if (!this->readHeader(pFile, size) || ROWS != size[0] || COLS != size[1])
        {
            fclose(pFile);
            return false;
        }
        size[2] += this->slices();
        std::fseek(pFile, 0, SEEK_SET);
        for (unsigned int i = 0; i < 3; i++)
        {
          std::fwrite(&size[i], sizeof size[i], 1, pFile);
        }
        // move pointer to end of file, and append data
        std::fseek(pFile, 0, SEEK_END);
        size_t nActuallyWritten = std::fwrite(&data[0], sizeof data[0], this->slices(), pFile);
        std::fclose(pFile);
        return nActuallyWritten == this->slices();
    }
    /**
    * Load data from a file.
    * \param filename
    * \return true if successful.
    */
    bool load(std::string const &filename)
    {
        std::FILE *pFile = std::fopen(filename.c_str(), "rb");
        if (pFile == nullptr)
        {
            return false;
        }

        std::vector<unsigned int> size(3);
        readHeader(pFile, size); // nRows, nCols, nSlices

        size_t nSlices(size[2]);
        data.resize(nSlices);
        std::fclose(pFile);
        assert(ROWS == size[0]);
        assert(COLS == size[1]);
    #ifndef NDEBUG
        size_t nActuallyRead = std::fread(&data[0], sizeof data[0], nSlices, pFile);
        assert(nActuallyRead == nSlices);
    #endif
        return true;
    }

  private:
    // Read header. Format is rows cols slices
    bool readHeader(FILE *pFile, std::vector<unsigned int> &size)
    {
        for (unsigned int i = 0; i < 3; i++)
        {
            if (std::fread(&size[i], sizeof size[i], 1, pFile) != 1)
            {
                return false;
            }
        }
        return true;
    }
};
} // namespace CalibUtils
