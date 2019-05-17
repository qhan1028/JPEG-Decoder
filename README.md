# JPEG Decoder

* R07944007 林良翰

## Environment

* macOS High Sierra 10.13.6
* g++

  ```
  g++ --version
  ```
  
  ```
  Configured with: --prefix=/Library/Developer/CommandLineTools/usr --with-gxx-include-dir=/usr/include/c++/4.2.1
  Apple LLVM version 10.0.0 (clang-1000.10.44.2)
  Target: x86_64-apple-darwin17.7.0
  Thread model: posix
  InstalledDir: /Library/Developer/CommandLineTools/usr/bin
  ```

## Usage

* Compile

  ```
  make
  ```

* Execute

  ```
  ./jpeg-decoder [jpeg_filename]
  ```

* Execute with Debug Mode

  ```
  ./jpeg-decoder [jpeg_filename] --debug
  ```