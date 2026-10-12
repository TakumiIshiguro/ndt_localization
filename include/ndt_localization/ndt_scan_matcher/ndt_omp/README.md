# ndt_omp

<!-->

cspell:ignore Kenji, Koide
<-->

The codes in this directory are derived from <https://github.com/tier4/ndt_omp>, which is a fork of <https://github.com/koide3/ndt_omp>.

We use this code in accordance with the LICENSE, and we have directly confirmed this with Dr. Kenji Koide.

We sincerely appreciate Dr. Koide’s support and contributions.

## Vendoring note (ndt_localization)

Copied unchanged from autoware_core `localization/autoware_ndt_scan_matcher` (commit
e0a8ab847413b73d9a48e13369943ae90cd4859a, 2026-10-08). The only edit is the include prefix
`autoware/ndt_scan_matcher/ndt_omp/` -> `ndt_localization/ndt_scan_matcher/ndt_omp/`.
`estimate_covariance.*` is not vendored because this package uses the fixed covariance only.
