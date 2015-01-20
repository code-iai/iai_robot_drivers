#ifndef IAI_KMS_40_DRIVER_PARSER_H_
#define IAI_KMS_40_DRIVER_PARSER_H_

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include <string>
#include <iai_kms_40_driver/wrench.hpp>

namespace iai_kms_40_driver
{
  bool parse_wrench(const std::string& msg, Wrench& wrench)
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
