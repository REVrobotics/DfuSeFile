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
        for (int i=0;i<n;i++) {
            crc32 = crc32_table[((crc32)^(static_cast<uint8_t>(s[i])))&0xff]^((crc32)>>8);
        }
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
        for (int i = 0; i < 255; i++) {
            if (m_prefix.Name[i] == '\1') {
                m_prefix.Name[i] = '\0';
            }
        }
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
                  11, // Filesize, start at prefix size
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

        //outfile << m_suffix;
        outfile.write((char*)&m_suffix.DeviceVersion, 2);
        outfile.write((char*)&m_suffix.Product, 2);
        outfile.write((char*)&m_suffix.Vendor, 2);
        outfile.write((char*)&m_suffix.DfuFormat, 2);
        outfile.write((char*)m_suffix.Ufd, 3);
        outfile.write((char*)&m_suffix.Length, 1);

        bool success = outfile.good();
        outfile.close();

        std::cout << "File CRC: 0x" << m_suffix.Crc32 << std::endl;

        std::ifstream infile(filename, std::ios::in | std::ios::binary);
        
        detail::crcstreambuf crcstreambuf(0xedb88320);
        std::iostream crcstream(&crcstreambuf);
        //uint32_t result1 = 1234;

        //crcstream.read((char*)&result1, 4);
        //std::cout << "File CRC Check: 0x" << result1 << std::endl;
        //crcstream << infile.rdbuf();

        uint32_t result1 = 0xFFFFFFFF;

        // get length of file:
        infile.seekg (0, infile.end);
        int length = infile.tellg();
        infile.seekg (0, infile.beg);

        for(int i=0; i<length;i++) {
            //const char tmp12 = (infile.get());
            //crcstream.write(&tmp12, 1);
            //result1 = detail::crc32(result1, &tmp12, 1);
            crcstream << static_cast<uint8_t>(infile.get());
        }

        crcstream.read((char*)&result1, 4);

        std::cout << "File CRC Check: 0x" << result1 << std::endl;
        std::cout << "Infile eof " << (int)infile.eof() << std::endl;

        infile.close();

        std::ofstream outfile2(filename, std::ios::out | std::ios::binary | std::ios::app);
        outfile2.seekp(0, outfile2.end);
        outfile2.write((char*)&result1, 4);
        outfile2.close();

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

} // namespace dfusefile
