# Legacy Source Provenance

The architecture review and rewrite design are based on the original [Flix repository](https://github.com/okalachev/flix) at revision `1f5b268271536e45a9377d1c5317d3eb6f3c3bba`.

Local reference checkout: `/Users/abuhamzah/Dev/erik/flix`, a sibling of this repository. The reviewed source baseline is [the pinned revision](https://github.com/okalachev/flix/tree/1f5b268271536e45a9377d1c5317d3eb6f3c3bba). Source links in the architecture report point to this immutable revision.

The transferred review, specification, and plan were written on 2026-10-05. The new repository has its own Git history. Legacy C++ source has not been copied into its implementation tree.

When adding comparison fixtures or copied reference excerpts, preserve original copyright notices, record their source path and revision, and verify applicable licensing. Do not infer that a file without an explicit license grants a particular redistribution license.

Hardware and timing observations in the review refer to the stated host probes and static inspection. No ESP32 or flight qualification has been completed for the Rust application.

## README visual assets

The root README adopts the original centered header layout and side-by-side hardware gallery, with a new Flix RS project logo. Six unchanged visual assets are copied from the pinned upstream revision into `docs/img/`. Their source, attribution, license uncertainty, and SHA-256 hashes are recorded in [the asset inventory](img/README.md). The upstream hardware photographs and simulator screenshot do not represent physical qualification or a screenshot of the Rust browser viewer.
