/*
 * Copyright (c) 2015, Georg Bartels (georg.bartels@cs.uni-bremen.de)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IAI_KMS_40_DRIVER_PARSER_H_
#define IAI_KMS_40_DRIVER_PARSER_H_

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include <string>
#include <iai_kms_40_driver/wrench.hpp>

namespace iai_kms_40_driver
{
  inline bool parse_wrench(const std::string& msg, Wrench& wrench)
  {
    using boost::spirit::qi::double_;
    using boost::spirit::qi::long_;
    using boost::spirit::qi::_1;
    using boost::spirit::qi::phrase_parse;
    using boost::spirit::ascii::space;
    using boost::phoenix::ref;

    std::string::const_iterator first = msg.begin();
    std::string::const_iterator last = msg.end();

    bool r = phrase_parse(first, last, 

        //  Begin grammar
        (
                "F={" >> double_[ref(wrench.fx_) = _1] >> ','
                      >> double_[ref(wrench.fy_) = _1] >> ','
                      >> double_[ref(wrench.fz_) = _1] >> ','
                      >> double_[ref(wrench.tx_) = _1] >> ','
                      >> double_[ref(wrench.ty_) = _1] >> ','
                      >> double_[ref(wrench.tz_) = _1] >> "},"
                      >> long_[ref(wrench.timestamp_) = _1]

        ),
        //  End grammar

        space);

    // fail if we did not get a full match
    return r && (first == last);
  }
}
#endif // IAI_KMS_40_DRIVER_PARSER_H_
