/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Func;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import static android.R.attr.data;

public class DebugFile {
    private Writer fileWriter;

    private String pathForFile;

    public DebugFile(String pathToFile) {
        pathForFile = pathToFile;
    }

    public void init() {
        try {
            FileWriter writer = new FileWriter(pathForFile, true);
            fileWriter = new BufferedWriter(writer);
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }

    public void write(String caption, String data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, boolean data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, byte data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, short data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, int data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, long data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, char data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, float data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }
    public void write(String caption, double data) {
        try {
            fileWriter.write("[" + currentDateAndTime() + "] " + caption + ": " + data + "\n");
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }
    }

    public void stop() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            throw new RuntimeException("Cannot close file", e);
        }
    }

    public String currentDateAndTime() {
        SimpleDateFormat format= new SimpleDateFormat("MM/d/yy, kk:mm:ss.SSS", Locale.getDefault());

        return format.format(new Date());
    }
}