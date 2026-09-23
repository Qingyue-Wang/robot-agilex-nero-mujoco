# SPDX-License-Identifier: Apache-2.0
"""Shared Robonix provider for the fixed-base Nero arm (MuJoCo Native)."""

from robonix_api import Primitive


# The deploy manifest (robonix_manifest.yaml) names this instance `nero_arm`;
# the id here must match that name verbatim.
nero_arm = Primitive(
    id="nero_arm",
    namespace="robonix/primitive/arm",
)
