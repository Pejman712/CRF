@ingroup group_std_json_converters

This is the module responsible of handling the convertion between special values
of doubles, namely:
    - std::numeric_limits<double>::infinity()
    - -std::numeric_limits<double>::infinity()
    - std::numeric_limits<double>::quier_NaN()
and their CRF-specific encodings into strings inside json namely:
    - "inf"
    - "-inf"
    - "nan"

Jsons are handled with nlohmann::json library.
