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

#include "DfuSeFile.h"

#include <iostream>

int main() {
    dfuse::DFUFile myFile("TestDFU.dfu");

    if (myFile) {
        std::cout << "Vendor: 0x" << std::hex << myFile.Vendor() << " Product: 0x" << std::hex << myFile.Product() << " Device Version: 0x" << std::hex << myFile.DeviceVersion() << std::endl;
        std::cout << "Number of Targets: " << myFile.Images().size() << std::endl;

        for (auto image : myFile.Images()) {
            if (image) {
                std::cout << "\t Id: " << image.Id() << " Name: " << image.Name() << " Size: " << image.Size() 
                          << " consisting of " << image.Elements().size() << " element(s)." << std::endl;
                for (auto element : image.Elements()) {
                    std::cout << "\t\t Element Address: 0x" << std::hex << element.Address() << " Size: " << element.Size() << std::endl;
                }
            } else {
                std::cout << "\t INVALID IMAGE!" << std::endl;
            }
        }
        return 0;
    }
    return -1;
}
