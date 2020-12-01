/*
 * Copyright (c) 2019-2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Library to read and write from DFU files based on the ST spec UM0391
 * 
 * Today this library copies everything into memory for speed of implementing
 * this library, and since the largest expected image is 256K this is not a
 * big deal. Future versions of this library could keep the bulk parts of
 * the image stored in their respective files.
 */

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>
#include <limits>

namespace dfuse {

constexpr uint16_t FormatVersion = 0x11a;

namespace detail {
    uint32_t crc32(uint32_t crc, const uint8_t *buf, size_t len);
/*
 * This class allows writing stream data through std::ostream and
 * calculating a CRC32 value from it. All data is accepted and will
 * never overflow. Data is immediately calculated then discarded.
 * 
 * Reading from the streambuf will return the result, always read
 * 4 bytes at a time. The buffer is a circular buffer around the CRC32
 * 
 */
class crcstreambuf : public std::streambuf {
public:
    crcstreambuf(uint32_t polynomial, bool reflect_begin = true, bool reflect_end = true, uint32_t initialValue = 0xFFFFFFFF) 
        : begin_(&reinterpret_cast<const char&>(crc32)),
          end_(&reinterpret_cast<const char&>(crc32) + 4),
          current_(&reinterpret_cast<const char&>(crc32)),
          reflect_begin(false),
          reflect_end(false),
          crc32(initialValue)
    {
        crc32_make_table(polynomial);
    }
 
private:
    int_type underflow() {
        if (current_ == end_) {
            return traits_type::eof();
        }
        return traits_type::to_int_type(*current_);
    }
    int_type uflow() {
        if (current_ == end_) {
            return traits_type::eof();
        }
        int_type result = traits_type::to_int_type(*current_++);
        if (current_ == end_) {
            current_ = begin_;
        }
        return result;
    }
    int_type pbackfail(int_type ch) {
        if (current_ == begin_ || (ch != traits_type::eof() && ch != current_[-1])) {
            return traits_type::eof();
        }
        return traits_type::to_int_type(*--current_);
    }
    std::streamsize showmanyc() {
        return end_ - current_;
    }
    std::streamsize xsputn(const char* s, std::streamsize n) {
        crc32_calculate(s, n);
		//for (int i = n-1; i>=0; i--) {
		//	crc32_calculate(&s[i], 1);
		//}
        return n;
    }
    
    void crc32_make_table(uint32_t poly) {
        for (int i = 0; i < 256; i++) {
            uint32_t crc = i;
            for (int j = 8; j > 0; j--) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ poly;
                } else {
                crc >>= 1;
                }
            }
            crc32_table[i] = crc;
        }
    }

    void crc32_calculate(const char* s, std::streamsize n) {
        for (int i = 0; i < n; i++) {
            uint8_t val = static_cast<uint8_t>(*s++);
            crc32 = ((crc32 >> 8) & 0x00FFFFFF) ^ crc32_table[(crc32 ^ val) & 0xFF];
        }
        crc32 = (crc32 ^ 0xFFFFFFFF);
    }
 
    const char * const begin_;
    const char * const end_;
    const char * current_;
    uint32_t crc32;
    uint32_t crc32_table[256];
    bool reflect_begin;
    bool reflect_end;
};
}

class DFUTarget {
public:
    DFUTarget(uint32_t address, uint8_t* data, uint32_t size) 
        : m_prefix( {address, size} ) {
        m_elements.resize(size);
        std::memcpy(m_elements.data(), data, size);
    }
    DFUTarget(uint32_t address, std::string inputFile) {
        m_prefix.Address = address;
        std::ifstream in(inputFile, std::ios::in | std::ios::binary);

        in.ignore( std::numeric_limits<std::streamsize>::max() );
        std::streamsize length = in.gcount();
        if (length > std::numeric_limits<uint32_t>::max()) {
            in.close();
            throw "File too large!";
        }
        m_prefix.Size = static_cast<uint32_t>(length);
        in.clear();   //  Since ignore will have set eof.
        in.seekg( 0, std::ios_base::beg );

        m_elements.resize(m_prefix.Size);
        in.read((char*)m_elements.data(), m_prefix.Size);
        in.close();
    }
    DFUTarget(uint32_t address, uint8_t value, uint32_t size) 
        : m_prefix( {address, size} ),
          m_elements(size, value) { }
    DFUTarget() {}
    uint32_t Address() const { return m_prefix.Address; }
    uint32_t Size() const { return m_prefix.Size; }
    const std::vector<uint8_t>& Data() const { return m_elements; }
private:
    friend std::istream & operator >> (std::istream &in,  DFUTarget &obj) {
        in >> obj.m_prefix;

        // We will stream in the elements later, save the stream location
        obj.m_elements.resize(obj.m_prefix.Size);
        in.read((char*)obj.m_elements.data(), obj.m_prefix.Size);

        //in.seekg(obj.m_prefix.Size, std::ios_base::cur);

        return in;
    }
    friend std::ostream & operator << (std::ostream &out,  DFUTarget &obj) {
        out << obj.m_prefix;

        out.write((char*)obj.m_elements.data(), obj.m_prefix.Size);

        return out;
    }
    struct Prefix {
        uint32_t Address;
        uint32_t Size;

        friend std::istream & operator >> (std::istream &in,  Prefix &obj) {
            //   <   little endian
            //   I   uint32_t    element address
            //   I   uint32_t    element size
            in.read((char*)&obj.Address, 4);
            in.read((char*)&obj.Size, 4);
            return in;
        }
        friend std::ostream & operator << (std::ostream &out,  Prefix &obj) {
            //   <   little endian
            //   I   uint32_t    element address
            //   I   uint32_t    element size
            out.write((char*)&obj.Address, 4);
            out.write((char*)&obj.Size, 4);
            return out;
        }
    };
    Prefix m_prefix;
    std::vector<uint8_t> m_elements;
};

namespace writer {
namespace detail {
class FileWriter {
public:
    FileWriter() {}
    virtual void Write(std::ofstream& out, const DFUTarget& target) =0;
    virtual std::unique_ptr<FileWriter> Clone() =0;
};

class BinWriter : public FileWriter {
public:
    BinWriter() { }
    virtual void Write(std::ofstream& out, const DFUTarget& target) override {
        out.write((const char*)target.Data().data(), target.Data().size());
    }
    virtual std::unique_ptr<FileWriter> Clone() override {return std::make_unique<BinWriter>( *this ); }
};
}

detail::BinWriter Bin;

} // namespace writer

class DFUImage {
public:
    DFUImage(uint8_t id, std::string targetName = "") {
        m_prefix.AltSetting = id;
        m_prefix.IsNamed = !targetName.empty();
        std::strncpy(m_prefix.Name, targetName.c_str(), 255);
        std::memcpy(m_prefix.Signature, "Target", 6);
        m_prefix.Size = 0;
        m_valid = false;
    }
    DFUImage() : m_valid(false) { }
    int Id() { return m_prefix.AltSetting; }
    const char* Name() const { return m_prefix.Name; }
    int Size() const { return m_prefix.Size; }
    const std::vector<DFUTarget>& Elements() const { return m_targets; }
    void Write(const std::string filename, writer::detail::FileWriter& writer) const {
        std::ofstream outputFile(filename, std::ofstream::binary);
        auto fw = writer.Clone();
        fw->Write(outputFile, m_targets[0]);
        outputFile.close();
    }

    operator bool() const {return m_valid;}
    bool operator!() const {return !m_valid;}

    void AddTarget(const DFUTarget& target) {
        m_targets.push_back(target);
        // Add size of whole target size (image + address (32bit) + size (32bit))
        m_prefix.Size += target.Size() + 8;
        m_prefix.Elements++;
        m_valid = true;
    }

private:
    friend std::istream & operator >> (std::istream &in,  DFUImage &obj) {
        obj.m_valid = false;
        in >> obj.m_prefix;

        if (!in || std::memcmp(obj.m_prefix.Signature,"Target",6) != 0) {
            return in;
        }

        obj.m_targets.resize(obj.m_prefix.Elements);
        
        for (DFUTarget& target : obj.m_targets) {
            in >> target;
            if (!in) {
                return in;
            }
        }

        obj.m_valid = true;
        return in;
    }
    friend std::ostream & operator << (std::ostream &out,  DFUImage &obj) {
        out << obj.m_prefix;
        
        for (DFUTarget& target : obj.m_targets) {
            out << target;
            if (!out) {
                break;
            }
        }

        return out;
    }

    struct Prefix {
        char Signature[6];
        uint8_t AltSetting;
        uint32_t IsNamed;
        char Name[255];
        uint32_t Size;
        uint32_t Elements;

        //   <   little endian
        //   6s      char[6]     signature   "Target"
        //   B       uint8_t     altsetting
        //   I       uint32_t    named       bool indicating if a name was used
        //   255s    char[255]   name        name of the target
        //   I       uint32_t    size        size of image (not incl prefix)
        //   I       uint32_t    elements    Number of elements in the image
        friend std::istream & operator >> (std::istream &in,  Prefix &obj) {
            in.read((char*)obj.Signature, 6);
            in.read((char*)&obj.AltSetting, 1);
            in.read((char*)&obj.IsNamed, 4);

            // TODO: Is this field always there
            in.read(obj.Name, 255);

            in.read((char*)&obj.Size, 4);
            in.read((char*)&obj.Elements, 4);
            return in;
        }
        friend std::ostream & operator << (std::ostream &out,  Prefix &obj) {
            out.write((char*)obj.Signature, 6);
            out.write((char*)&obj.AltSetting, 1);
            out.write((char*)&obj.IsNamed, 4);

            // TODO: Is this field always there
            out.write(obj.Name, 255);

            out.write((char*)&obj.Size, 4);
            out.write((char*)&obj.Elements, 4);
            return out;
        }
    };
    Prefix m_prefix;
    std::vector<DFUTarget> m_targets;
    bool m_valid;
};

class DFUFile {
public:
    DFUFile(uint16_t vendorId, uint16_t productId, uint16_t version)
            : m_valid(false),
              m_prefix({ 
                  {'D', 'f', 'u', 'S', 'e'},
                  1,
                  27, // Filesize, start at prefix + suffix sizes
                  0}),
              m_images(),
              m_suffix({
                  version,
                  productId,
                  vendorId,
                  FormatVersion,
                  {'U', 'F', 'D'},
                  16, // fixed value in spec
                  0}) {  }

    DFUFile(const char* filename) {
        std::ifstream dfuFile(filename, std::ios_base::binary);
        m_valid = false;

        if (!dfuFile) {
            // TODO: Throw an error
            return;
        }

        dfuFile >> m_prefix;

        if (!dfuFile || std::memcmp(m_prefix.Signature,"DfuSe",5) != 0) {
            // TODO: Throw an error
            return;
        }
        m_images.resize(m_prefix.Targets);

        for (DFUImage& image : m_images) {
            dfuFile >> image;
            if (!dfuFile || !image) {
                // TODO: Throw an error
                return;
            }
        }

        dfuFile >> m_suffix;

        // TODO: Check CRC
        m_valid = true;
        dfuFile.close();
    };

    void AddImage(const DFUImage& image) {
        if (image) {
            detail::crcstreambuf crcstreambuf(0xedb88320);
            std::iostream crcstream(&crcstreambuf);
            m_images.push_back(image);
            m_valid = true;
            m_prefix.Targets++;

            // Size is image size (size of the image without prefix) + prefix (274 bytes)
            m_prefix.Size += image.Size() + 274;

            // Finally calculate CRC, this is done over the *entire* image except the CRC field
            // We can simply stream the crc32 from m_prefix and the stored DFUImages, but manually for m_suffix
            crcstream << m_prefix;
            for (const DFUImage& image : m_images) {
                crcstream << image;
            }

            crcstream.write((char*)&m_suffix.DeviceVersion, 2);
            crcstream.write((char*)&m_suffix.Product, 2);
            crcstream.write((char*)&m_suffix.Vendor, 2);
            crcstream.write((char*)&m_suffix.DfuFormat, 2);
            crcstream.write((char*)m_suffix.Ufd, 3);
            crcstream.write((char*)&m_suffix.Length, 1);

            crcstream.read((char*)&m_suffix.Crc32, 4);
        }
    }

    uint32_t Write(std::string filename) {
        if (!m_valid) {
            return 1;
        }

        std::ofstream outfile(filename, std::ios::out | std::ios::binary);

        outfile << m_prefix;

        for (DFUImage& image : m_images) {
            outfile << image;
        }

        outfile << m_suffix;

        bool success = outfile.good();
        outfile.close();

        std::cout << "File CRC: 0x" << m_suffix.Crc32 << std::endl;

        std::ifstream infile(filename, std::ios::in | std::ios::binary);
        
        //detail::crcstreambuf crcstreambuf(0xedb88320);
        //std::iostream crcstream(&crcstreambuf);
        //uint32_t result1 = 1234;

        //crcstream.read((char*)&result1, 4);
        //std::cout << "File CRC Check: 0x" << result1 << std::endl;
        //crcstream << infile.rdbuf();

        uint32_t result1 = 0xFFFFFFFF;
        while (!infile.eof()) {
            uint8_t tmp12 = static_cast<uint8_t>(infile.get());
            result1 = detail::crc32(result1, &tmp12, 1);
            //crcstream << infile.get();
        }

        //crcstream.read((char*)&result1, 4);

        std::cout << "File CRC Check: 0x" << result1 << std::endl;
        std::cout << "Infile eof " << (int)infile.eof() << std::endl;

        return success ? 0 : 1;
    }

    operator bool() const {return m_valid;}
    bool operator!() const {return !m_valid;}

    unsigned int FileFormatVersion() { return m_prefix.Version; }
    unsigned int Vendor() { return m_suffix.Vendor; }
    unsigned int Product() { return m_suffix.Product; }
    unsigned int DeviceVersion() { return m_suffix.DeviceVersion; }
    const std::vector<DFUImage>& Images() const { return m_images; }
    uint32_t Crc() { return m_suffix.Crc32; }

private:
    DFUFile() = delete;
    bool m_valid;

    struct Prefix {
        uint8_t Signature[5];
        uint8_t Version;
        uint32_t Size;
        uint8_t Targets;

        //   <   little endian
        //   5s  char[5]     signature   "DfuSe"
        //   B   uint8_t     version     1
        //   I   uint32_t    size        Size of the DFU file (not including suffix)
        //   B   uint8_t     targets     Number of targets
        friend std::istream & operator >> (std::istream &in,  Prefix &obj) {
            in.read((char*)obj.Signature, 5);
            in.read((char*)&obj.Version,1);
            in.read((char*)&obj.Size,4);
            in.read((char*)&obj.Targets,1);
            return in;
        }
        friend std::ostream & operator << (std::ostream &out, Prefix &obj) {
            out.write((char*)obj.Signature, 5);
            out.write((char*)&obj.Version,1);
            out.write((char*)&obj.Size,4);
            out.write((char*)&obj.Targets,1);
            return out;
        }
    };

    Prefix m_prefix;

    std::vector<DFUImage> m_images;

    struct Suffix {
        uint16_t DeviceVersion;
        uint16_t Product;
        uint16_t Vendor;
        uint16_t DfuFormat;
        uint8_t Ufd[3];
        uint8_t Length;
        uint32_t Crc32;

        //   <   little endian
        //   H   uint16_t    device  Firmware version
        //   H   uint16_t    product
        //   H   uint16_t    vendor
        //   H   uint16_t    dfu     0x11a   (DFU file format version)
        //   3s  char[3]     ufd     'UFD'
        //   B   uint8_t     len     16
        //   I   uint32_t    crc32
        friend std::istream & operator >> (std::istream &in,  Suffix &obj) {
            in.read((char*)&obj.DeviceVersion, 2);
            in.read((char*)&obj.Product, 2);
            in.read((char*)&obj.Vendor, 2);
            in.read((char*)&obj.DfuFormat, 2);
            in.read((char*)obj.Ufd, 3);
            in.read((char*)&obj.Length, 1);
            in.read((char*)&obj.Crc32, 4);
            return in;
        }
        friend std::ostream & operator << (std::ostream &out, Suffix &obj) {
            out.write((char*)&obj.DeviceVersion, 2);
            out.write((char*)&obj.Product, 2);
            out.write((char*)&obj.Vendor, 2);
            out.write((char*)&obj.DfuFormat, 2);
            out.write((char*)obj.Ufd, 3);
            out.write((char*)&obj.Length, 1);
            out.write((char*)&obj.Crc32, 4);
            return out;
        }
    };
    Suffix m_suffix;
};

#ifndef REMOVE_THIS
namespace detail {
//extracted from https://github.com/madler/zlib/blob/master/crc32.c
uint32_t crc_tab[256] = {
0x0000000, 0x77073096, 0xee0e612c, 0x990951ba,
0x76dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
0xedb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
0x9b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de,
0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940,
0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116,
0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
0x76dc4190, 0x1db7106, 0x98d220bc, 0xefd5102a,
0x71b18589, 0x6b6b51f, 0x9fbfe4a5, 0xe8b8d433,
0x7807c9a2, 0xf00f934, 0x9609a88e, 0xe10e9818,
0x7f6a0dbb, 0x86d3d2d, 0x91646c97, 0xe6635c01,
0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c,
0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2,
0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086,
0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4,
0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
0xedb88320, 0x9abfb3b6, 0x3b6e20c, 0x74b1d29a,
0xead54739, 0x9dd277af, 0x4db2615, 0x73dc1683,
0xe3630b12, 0x94643b84, 0xd6d6a3e, 0x7a6a5aa8,
0xe40ecf0b, 0x9309ff9d, 0xa00ae27, 0x7d079eb1,
0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe,
0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252,
0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60,
0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04,
0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x26d930a,
0x9c0906a9, 0xeb0e363f, 0x72076785, 0x5005713,
0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0xcb61b38,
0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0xbdbdf21,
0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e,
0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c,
0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0,
0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6,
0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};

uint32_t crc32(uint32_t crc, const uint8_t *buf, size_t len)
{
   for (int i = 0; i < len; i++) {
      crc = ((crc >> 8) & 0x00FFFFFF) ^ crc_tab[(crc ^ *buf++) & 0xFF];
   }
   return (crc ^ 0xFFFFFFFF);
}

}
#endif

} // namespace dfusefile
