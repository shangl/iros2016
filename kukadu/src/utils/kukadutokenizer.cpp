///////////////////////////////////////////////////////////////////////////////
// Tokenizer.cpp
// =============
// General purpose string tokenizer (C++ string version)
//
// The default delimiters are space(" "), tab(\t, \v), newline(\n),
// carriage return(\r), and form feed(\f).
// If you want to use different delimiters, then use setDelimiter() to override
// the delimiters. Note that the delimiter string can hold multiple characters.
//
//  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2005-05-25
// UPDATED: 2011-03-08
///////////////////////////////////////////////////////////////////////////////

#include "kukadutokenizer.hpp"

namespace kukadu {

    ///////////////////////////////////////////////////////////////////////////////
    // constructor
    ///////////////////////////////////////////////////////////////////////////////
    KukaduTokenizer::KukaduTokenizer() : buffer(""), token(""), delimiter(DEFAULT_DELIMITER) {
        currPos = buffer.begin();
        lastToken = "";
        useLastToken = false;
        tokenIdx = -1;
    }

    KukaduTokenizer::KukaduTokenizer(const std::string& str, const std::string& delimiter) : buffer(str), token(""), delimiter(delimiter) {
        currPos = buffer.begin();
        lastToken = "";
        useLastToken = false;
        tokenIdx = -1;
    }



    ///////////////////////////////////////////////////////////////////////////////
    // destructor
    ///////////////////////////////////////////////////////////////////////////////
    KukaduTokenizer::~KukaduTokenizer() {
    }



    ///////////////////////////////////////////////////////////////////////////////
    // reset string buffer, delimiter and the currsor position
    ///////////////////////////////////////////////////////////////////////////////
    void KukaduTokenizer::set(const std::string& str, const std::string& delimiter) {
        this->buffer = str;
        this->delimiter = delimiter;
        this->currPos = buffer.begin();
    }

    void KukaduTokenizer::setString(const std::string& str) {
        this->buffer = str;
        this->currPos = buffer.begin();
    }

    void KukaduTokenizer::setDelimiter(const std::string& delimiter) {
        this->delimiter = delimiter;
        this->currPos = buffer.begin();
    }

    int KukaduTokenizer::getTokenIdx() {
        return tokenIdx;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // return the next token
    // If cannot find a token anymore, return "".
    ///////////////////////////////////////////////////////////////////////////////
    std::string KukaduTokenizer::next() {
        ++tokenIdx;
        if(useLastToken) {
            useLastToken = false;
            return lastToken;
        } else {
            if(buffer.size() <= 0) return "";           // skip if buffer is empty

            token.clear();                              // reset token string

            this->skipDelimiter();                      // skip leading delimiters

            // append each char to token string until it meets delimiter
            while(currPos != buffer.end() && !isDelimiter(*currPos)) {
                token += *currPos;
                ++currPos;
            }
            return (lastToken = token);
        }
        return "";
    }

    void KukaduTokenizer::putBackLast() {
        useLastToken = true;
        if(tokenIdx > 0)
            tokenIdx--;
    }



    ///////////////////////////////////////////////////////////////////////////////
    // skip ang leading delimiters
    ///////////////////////////////////////////////////////////////////////////////
    void KukaduTokenizer::skipDelimiter() {
        while(currPos != buffer.end() && isDelimiter(*currPos))
            ++currPos;
    }



    ///////////////////////////////////////////////////////////////////////////////
    // return true if the current character is delimiter
    ///////////////////////////////////////////////////////////////////////////////
    bool KukaduTokenizer::isDelimiter(char c) {
        return (delimiter.find(c) != std::string::npos);
    }



    ///////////////////////////////////////////////////////////////////////////////
    // split the input string into multiple tokens
    // This function scans tokens from the current cursor position.
    ///////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> KukaduTokenizer::split() {
        std::vector<std::string> tokens;
        std::string token;
        while((token = this->next()) != "") {
            tokens.push_back(token);
        }

        return tokens;
    }

}
