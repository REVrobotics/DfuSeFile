/*
 * Copyright (c) 2019 REV Robotics
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

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>

namespace dfuse {

class DFUTarget {
public:
    uint32_t Address() { return m_prefix.Address; }
    int Size() { return m_prefix.Size; }
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
    };
    Prefix m_prefix;
    std::vector<uint8_t> m_elements;
};

namespace writer {

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

BinWriter Bin;

} // namespace writer

class DFUImage {
public:
    int Id() { return m_prefix.AltSetting; }
    const char* Name() { return m_prefix.Name; }
    int Size() { return m_prefix.Size; }
    const std::vector<DFUTarget>& Elements() const { return m_targets; }
    void Write(const std::string filename, const int elementIndex, writer::FileWriter& writer) {
        std::ofstream outputFile(filename, std::ofstream::binary);
        auto fw = writer.Clone();
        fw->Write(outputFile, m_targets[elementIndex]);
        outputFile.close();
    }

    operator bool() const {return m_valid;}
    bool operator!() const {return !m_valid;}

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
    struct Prefix {
        uint8_t Signature[6];
        uint8_t AltSetting;
        uint32_t IsNamed;
        char Name[255];
        uint32_t Size;
        uint32_t Elements;

        friend std::istream & operator >> (std::istream &in,  Prefix &obj) {
            //   <   little endian
            //   6s      char[6]     signature   "Target"
            //   B       uint8_t     altsetting
            //   I       uint32_t    named       bool indicating if a name was used
            //   255s    char[255]   name        name of the target
            //   I       uint32_t    size        size of image (not incl prefix)
            //   I       uint32_t    elements    Number of elements in the image
            in.read((char*)obj.Signature, 6);
            in.read((char*)&obj.AltSetting, 1);
            in.read((char*)&obj.IsNamed, 4);

            // TODO: Is this field always there
            in.read(obj.Name, 255);

            in.read((char*)&obj.Size, 4);
            in.read((char*)&obj.Elements, 4);
            return in;
        }
    };
    Prefix m_prefix;
    std::vector<DFUTarget> m_targets;
    bool m_valid;
};

class DFUFile {
public:
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

    //uint32_t Write(std::string filename) {
    //    return 0;
    //}

    operator bool() const {return m_valid;}
    bool operator!() const {return !m_valid;}

    unsigned int FileFormatVersion() { return m_prefix.Version; }
    unsigned int Vendor() { return m_suffix.Vendor; }
    unsigned int Product() { return m_suffix.Product; }
    unsigned int DeviceVersion() { return m_suffix.DeviceVersion; }
    const std::vector<DFUImage>& Images() const { return m_images; }
    uint32_t Crc() { return m_suffix.Crc32; }

private:
    DFUFile() {};
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
    };
    Suffix m_suffix;
};

} // namespace dfusefile
