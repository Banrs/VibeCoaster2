"""Blender 5.2-compatible / Blender MCP safe-mode environment kit.
Root precreates output directories. Execute the entire script through the guarded
MCP tool; it does not read files, install add-ons, run handlers, or contact services.
Only datablocks tagged VC_ENVKIT are replaced on rerun. The final JSON is the
manifest for the caller to persist. No runtime mesh or numerical source is changed.
"""
import bpy
import bmesh
import math
import json
from mathutils import Vector

OWNER = "VC_ENVKIT"
SOURCE_BLEND = "D:/Coding/Codex/Vibecoasterjs/native/art/source/environment/EnvironmentKit.blend"
EXPORT_DIR = "D:/Coding/Codex/Vibecoasterjs/native/art/exports/environment/"
SUPPORT_REFERENCE = {'sourceSave': 'D:/Coding/Codex/Vibecoasterjs/native/unreal/Saved/RuntimeVerification/20260907-folded-final/canyon42-profile/Saved/Designs/Accepted.vcdesign', 'sourceSha256': '26656c8b0aa12a6536513dc811a67576932c1caef57f0bdeeee41aa24dc6e336', 'saveGenerationVersion': '0.7.0-folded.1', 'integrity': 'COASTER5 byte count and FNV-1a checksum verified; source is previously accepted runtime design; not a new physics revalidation', 'support': {'index': 72, 'base': [-306.2229675105089, -1431.908707754239, 196.38878146804197], 'top': [-306.2229675105089, -1431.908707754239, 270.44386439600595], 'attachment': [-303.5249156445465, -1420.1065656446128, 271.63967280572825], 'trackDistance': 2880.0, 'members': [{'base': [-301.53284711168595, -1427.8367509708623, 194.84044399229538], 'top': [-301.53284711168595, -1427.8367509708623, 197.27652372346807], 'radiusBase': 2.3107187196154975, 'radiusTop': 2.3107187196154975, 'kind': 1, 'spineContact': False}, {'base': [-302.15101072713225, -1436.5988281530622, 194.8368619321739], 'top': [-302.15101072713225, -1436.5988281530622, 197.2729416633466], 'radiusBase': 2.3107187196154975, 'radiusTop': 2.3107187196154975, 'kind': 1, 'spineContact': False}, {'base': [-310.91308790933186, -1435.9806645376157, 194.834027231412], 'top': [-310.91308790933186, -1435.9806645376157, 197.2701069625847], 'radiusBase': 2.3107187196154975, 'radiusTop': 2.3107187196154975, 'kind': 1, 'spineContact': False}, {'base': [-310.2949242938856, -1427.2185873554158, 194.83754312620914], 'top': [-310.2949242938856, -1427.2185873554158, 197.27362285738184], 'radiusBase': 2.3107187196154975, 'radiusTop': 2.3107187196154975, 'kind': 1, 'spineContact': False}, {'base': [-301.53284711168595, -1427.8367509708623, 197.27652372346807], 'top': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'radiusBase': 0.48292118244152077, 'radiusTop': 0.4310019034167334, 'kind': 0, 'spineContact': False}, {'base': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'top': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-301.53284711168595, -1427.8367509708623, 197.27652372346807], 'top': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-302.15101072713225, -1436.5988281530622, 197.2729416633466], 'top': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-302.15101072713225, -1436.5988281530622, 197.2729416633466], 'top': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'radiusBase': 0.48292118244152077, 'radiusTop': 0.4310019034167334, 'kind': 0, 'spineContact': False}, {'base': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'top': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-302.15101072713225, -1436.5988281530622, 197.2729416633466], 'top': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-310.91308790933186, -1435.9806645376157, 197.2701069625847], 'top': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-310.91308790933186, -1435.9806645376157, 197.2701069625847], 'top': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'radiusBase': 0.48292118244152077, 'radiusTop': 0.4310019034167334, 'kind': 0, 'spineContact': False}, {'base': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'top': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-310.91308790933186, -1435.9806645376157, 197.2701069625847], 'top': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-310.2949242938856, -1427.2185873554158, 197.27362285738184], 'top': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-310.2949242938856, -1427.2185873554158, 197.27362285738184], 'top': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'radiusBase': 0.48292118244152077, 'radiusTop': 0.4310019034167334, 'kind': 0, 'spineContact': False}, {'base': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'top': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-310.2949242938856, -1427.2185873554158, 197.27362285738184], 'top': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-301.53284711168595, -1427.8367509708623, 197.27652372346807], 'top': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'top': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'radiusBase': 0.4310019034167334, 'radiusTop': 0.37908262439194595, 'kind': 0, 'spineContact': False}, {'base': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'top': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'top': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'top': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'top': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'radiusBase': 0.4310019034167334, 'radiusTop': 0.37908262439194595, 'kind': 0, 'spineContact': False}, {'base': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'top': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-302.79440578680277, -1435.8577593121104, 211.90712620987847], 'top': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'top': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'top': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'radiusBase': 0.4310019034167334, 'radiusTop': 0.37908262439194595, 'kind': 0, 'spineContact': False}, {'base': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'top': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-310.1720190683801, -1435.337269477945, 211.90485844926897], 'top': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'top': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'top': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'radiusBase': 0.4310019034167334, 'radiusTop': 0.37908262439194595, 'kind': 0, 'spineContact': False}, {'base': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'top': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-309.65152923421505, -1427.9596561963676, 211.90767116510668], 'top': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-302.2739159526377, -1428.480146030533, 211.90999185797568], 'top': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'top': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'radiusBase': 0.37908262439194595, 'radiusTop': 0.32716334536715863, 'kind': 0, 'spineContact': False}, {'base': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'top': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'top': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'top': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'top': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'radiusBase': 0.37908262439194595, 'radiusTop': 0.32716334536715863, 'kind': 0, 'spineContact': False}, {'base': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'top': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-303.4378008464733, -1435.1166904711586, 226.54131075641033], 'top': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'top': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'top': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'radiusBase': 0.37908262439194595, 'radiusTop': 0.32716334536715863, 'kind': 0, 'spineContact': False}, {'base': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'top': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-309.4309502274283, -1434.6938744182746, 226.5396099359532], 'top': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'top': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'top': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'radiusBase': 0.37908262439194595, 'radiusTop': 0.32716334536715863, 'kind': 0, 'spineContact': False}, {'base': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'top': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-309.0081341745445, -1428.7007250373194, 226.5417194728315], 'top': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-303.0149847935895, -1429.1235410902034, 226.54345999248324], 'top': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'top': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'radiusBase': 0.32716334536715863, 'radiusTop': 0.2752440663423712, 'kind': 0, 'spineContact': False}, {'base': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'top': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'top': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'top': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'top': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'radiusBase': 0.32716334536715863, 'radiusTop': 0.2752440663423712, 'kind': 0, 'spineContact': False}, {'base': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'top': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-304.0811959061438, -1434.3756216302068, 241.1754953029422], 'top': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'top': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'top': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'radiusBase': 0.32716334536715863, 'radiusTop': 0.2752440663423712, 'kind': 0, 'spineContact': False}, {'base': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'top': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-308.68988138647654, -1434.050479358604, 241.17436142263745], 'top': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'top': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'top': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'radiusBase': 0.32716334536715863, 'radiusTop': 0.2752440663423712, 'kind': 0, 'spineContact': False}, {'base': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'top': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-308.364739114874, -1429.4417938782713, 241.17576778055633], 'top': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-303.7560536345412, -1429.766936149874, 241.17692812699082], 'top': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'top': [-305.2381913164448, -1431.053726269215, 270.44386439600595], 'radiusBase': 0.2752440663423712, 'radiusTop': 0.22332478731758382, 'kind': 0, 'spineContact': False}, {'base': [-305.2381913164448, -1431.053726269215, 270.44386439600595], 'top': [-305.36798602548487, -1432.8934839483031, 270.44386439600595], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'top': [-305.36798602548487, -1432.8934839483031, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'top': [-305.2381913164448, -1431.053726269215, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'top': [-305.36798602548487, -1432.8934839483031, 270.44386439600595], 'radiusBase': 0.2752440663423712, 'radiusTop': 0.22332478731758382, 'kind': 0, 'spineContact': False}, {'base': [-305.36798602548487, -1432.8934839483031, 270.44386439600595], 'top': [-307.20774370457303, -1432.763689239263, 270.44386439600595], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-304.72459096581434, -1433.634552789255, 255.8096798494741], 'top': [-307.20774370457303, -1432.763689239263, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'top': [-305.36798602548487, -1432.8934839483031, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'top': [-307.20774370457303, -1432.763689239263, 270.44386439600595], 'radiusBase': 0.2752440663423712, 'radiusTop': 0.22332478731758382, 'kind': 0, 'spineContact': False}, {'base': [-307.20774370457303, -1432.763689239263, 270.44386439600595], 'top': [-307.07794899553295, -1430.9239315601749, 270.44386439600595], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-307.9488125455248, -1433.4070842989336, 255.8091129093217], 'top': [-307.07794899553295, -1430.9239315601749, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'top': [-307.20774370457303, -1432.763689239263, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'top': [-307.07794899553295, -1430.9239315601749, 270.44386439600595], 'radiusBase': 0.2752440663423712, 'radiusTop': 0.22332478731758382, 'kind': 0, 'spineContact': False}, {'base': [-307.07794899553295, -1430.9239315601749, 270.44386439600595], 'top': [-305.2381913164448, -1431.053726269215, 270.44386439600595], 'radiusBase': 0.167027541463982, 'radiusTop': 0.167027541463982, 'kind': 0, 'spineContact': False}, {'base': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'top': [-305.2381913164448, -1431.053726269215, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'top': [-307.07794899553295, -1430.9239315601749, 270.44386439600595], 'radiusBase': 0.1333247873175838, 'radiusTop': 0.1333247873175838, 'kind': 0, 'spineContact': False}, {'base': [-305.2381913164448, -1431.053726269215, 270.44386439600595], 'top': [-306.2229675105089, -1431.908707754239, 270.44386439600595], 'radiusBase': 0.2233247873175838, 'radiusTop': 0.2233247873175838, 'kind': 0, 'spineContact': False}, {'base': [-305.36798602548487, -1432.8934839483031, 270.44386439600595], 'top': [-306.2229675105089, -1431.908707754239, 270.44386439600595], 'radiusBase': 0.2233247873175838, 'radiusTop': 0.2233247873175838, 'kind': 0, 'spineContact': False}, {'base': [-307.20774370457303, -1432.763689239263, 270.44386439600595], 'top': [-306.2229675105089, -1431.908707754239, 270.44386439600595], 'radiusBase': 0.2233247873175838, 'radiusTop': 0.2233247873175838, 'kind': 0, 'spineContact': False}, {'base': [-307.07794899553295, -1430.9239315601749, 270.44386439600595], 'top': [-306.2229675105089, -1431.908707754239, 270.44386439600595], 'radiusBase': 0.2233247873175838, 'radiusTop': 0.2233247873175838, 'kind': 0, 'spineContact': False}, {'base': [-306.2229675105089, -1431.908707754239, 270.44386439600595], 'top': [-304.92323521171636, -1420.0960847332524, 272.1824366020219], 'radiusBase': 0.45, 'radiusTop': 0.24, 'kind': 0, 'spineContact': False}, {'base': [-304.49712247549303, -1430.4103312095444, 255.8103962614984], 'top': [-304.92323521171636, -1420.0960847332524, 272.1824366020219], 'radiusBase': 0.18, 'radiusTop': 0.18, 'kind': 0, 'spineContact': False}, {'base': [-307.7213440552035, -1430.182862719223, 255.80981608828114], 'top': [-304.92323521171636, -1420.0960847332524, 272.1824366020219], 'radiusBase': 0.18, 'radiusTop': 0.18, 'kind': 0, 'spineContact': False}, {'base': [-304.92323521171636, -1420.0960847332524, 272.1824366020219], 'top': [-303.5249156445465, -1420.1065656446128, 271.63967280572825], 'radiusBase': 0.18, 'radiusTop': 0.18, 'kind': 0, 'spineContact': True}]}}

# Preserve other agents' scene and selection. Our exports temporarily use our scene.
previous_scene = bpy.context.window.scene
old_scenes = list(bpy.data.scenes)
scratch_scene = None
if not any(s.get("asset_owner") != OWNER for s in old_scenes):
    scratch_scene = bpy.data.scenes.new(OWNER + "_TemporaryContext")
    scratch_scene["asset_owner"] = OWNER
    bpy.context.window.scene = scratch_scene
    previous_scene = None
for scene in old_scenes:
    if scene.get("asset_owner") == OWNER:
        if any(obj.get("asset_owner") != OWNER for obj in scene.objects):
            raise RuntimeError("Refusing to replace a kit scene containing foreign objects")
        if scene == previous_scene:
            previous_scene = next((s for s in bpy.data.scenes if s != scene and s.get("asset_owner") != OWNER), None)
        bpy.data.scenes.remove(scene)
for obj in list(bpy.data.objects):
    if obj.get("asset_owner") == OWNER:
        bpy.data.objects.remove(obj, do_unlink=True)
for coll in list(bpy.data.collections):
    if coll.get("asset_owner") == OWNER:
        bpy.data.collections.remove(coll)
for mesh in list(bpy.data.meshes):
    if mesh.get("asset_owner") == OWNER and mesh.users == 0:
        bpy.data.meshes.remove(mesh)
for data in list(bpy.data.cameras):
    if data.get("asset_owner") == OWNER and data.users == 0:
        bpy.data.cameras.remove(data)
for data in list(bpy.data.lights):
    if data.get("asset_owner") == OWNER and data.users == 0:
        bpy.data.lights.remove(data)
for data in list(bpy.data.worlds):
    if data.get("asset_owner") == OWNER and data.users == 0:
        bpy.data.worlds.remove(data)


def owned(data):
    data["asset_owner"] = OWNER
    return data


def make_scene(name):
    scene = owned(bpy.data.scenes.new(OWNER + "_" + name))
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    engines = [item.identifier for item in scene.render.bl_rna.properties["engine"].enum_items]
    preferred = next((name for name in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE", "BLENDER_WORKBENCH") if name in engines), None)
    if preferred is None:
        raise RuntimeError("No supported review renderer is registered: " + str(engines))
    scene.render.engine = preferred
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 1000
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.world = owned(bpy.data.worlds.new(OWNER + "_" + name + "World"))
    scene.world.use_nodes = True
    scene.world.node_tree.nodes.get("Background").inputs[0].default_value = (0.055, 0.065, 0.085, 1)
    scene.world.node_tree.nodes.get("Background").inputs[1].default_value = 0.45
    scene.view_settings.view_transform = "AgX"
    return scene


scene = make_scene("StationAndModules")
bpy.context.window.scene = scene
if scratch_scene is not None:
    bpy.data.scenes.remove(scratch_scene)
stage = owned(bpy.data.collections.new(OWNER + "_Stage"))
scene.collection.children.link(stage)
assets = []
materials = {}


def material(name, color, metallic=0.0, roughness=0.45):
    full = OWNER + "_" + name
    mat = bpy.data.materials.get(full)
    if mat is not None and mat.get("asset_owner") != OWNER:
        raise RuntimeError("Material name collision: " + full)
    if mat is None:
        mat = owned(bpy.data.materials.new(full))
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = tuple(color) + (1,)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    mat.diffuse_color = tuple(color) + (1,)
    materials[name] = mat
    return mat


material("Graphite", (0.065, 0.085, 0.11), 0.45, 0.34)
material("BlueSteel", (0.13, 0.24, 0.28), 0.55, 0.31)
material("MachinedSteel", (0.42, 0.48, 0.53), 0.82, 0.28)
material("DarkRecess", (0.022, 0.028, 0.034), 0.22, 0.5)
material("Concrete", (0.43, 0.415, 0.38), 0.0, 0.81)
material("ConcreteEdge", (0.32, 0.315, 0.295), 0.0, 0.8)
material("SafetyOchre", (0.69, 0.40, 0.065), 0.2, 0.52)
material("LightDiffuser", (0.73, 0.79, 0.77), 0.05, 0.34)
material("ReviewCoral", (0.51, 0.13, 0.065), 0.45, 0.38)


def asset(name, bounds, pivot, status="ENVELOPE_FIT_REQUIRES_IMPORT_REVIEW"):
    coll = owned(bpy.data.collections.new(OWNER + "_" + name))
    root = owned(bpy.data.objects.new(OWNER + "_" + name + "_ROOT", None))
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.35
    coll.objects.link(root)
    root["asset_name"] = name
    root["integration_status"] = status
    item = {"name": name, "collection": coll, "root": root, "bounds": bounds,
            "pivot": pivot, "status": status, "objects": []}
    assets.append(item)
    return item


def mesh_object(item, part_name, vertices, faces, material_name, smooth_sides=False):
    mesh = owned(bpy.data.meshes.new(OWNER + "_" + part_name + "_Mesh"))
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    mesh.materials.append(materials[material_name])
    obj = owned(bpy.data.objects.new(OWNER + "_" + part_name, mesh))
    item["collection"].objects.link(obj)
    obj.parent = item["root"]
    if smooth_sides:
        for face in mesh.polygons:
            face.use_smooth = len(face.vertices) == 4
    item["objects"].append(obj)
    return obj


def box(item, name, center, size, mat, bevel=0.008):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1)
    for vert in bm.verts:
        vert.co.x = vert.co.x * size[0] + center[0]
        vert.co.y = vert.co.y * size[1] + center[1]
        vert.co.z = vert.co.z * size[2] + center[2]
    if bevel > 0:
        bmesh.ops.bevel(bm, geom=list(bm.edges), offset=min(bevel, min(size) * 0.22), segments=3, affect="EDGES")
    bm.normal_update()
    mesh = owned(bpy.data.meshes.new(OWNER + "_" + name + "_Mesh"))
    bm.to_mesh(mesh)
    bm.free()
    mesh.materials.append(materials[mat])
    obj = owned(bpy.data.objects.new(OWNER + "_" + name, mesh))
    item["collection"].objects.link(obj)
    obj.parent = item["root"]
    # Flat principal faces and bevel strips retain clean architectural edges.
    item["objects"].append(obj)
    return obj


def cylinder(item, name, center, radius_base, radius_top, height, mat, sides=32):
    verts = []
    for z, radius in ((-height * 0.5, radius_base), (height * 0.5, radius_top)):
        for n in range(sides):
            angle = 2 * math.pi * n / sides
            verts.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle), center[2] + z))
    faces = [(n, (n + 1) % sides, (n + 1) % sides + sides, n + sides) for n in range(sides)]
    faces.extend([tuple(reversed(range(sides))), tuple(range(sides, 2 * sides))])
    return mesh_object(item, name, verts, faces, mat, True)


def bolt(item, name, xy, top, radius=0.018, depth=0.012):
    return cylinder(item, name, (xy[0], xy[1], top - depth / 2), radius, radius * 0.96, depth, "MachinedSteel", 6)


def platform(length=3.0, name="SM_StationPlatformPanel"):
    item = asset(name, [[-length/2, -1.925, -0.8], [length/2, 1.925, 0]],
                 "Top centre; inner platform edge is local -Y. Tile along +X. Rotate opposite-side instances 180 degrees about Z.")
    box(item, name + "_ConcreteCore", (0, 0, -0.415), (length - 0.008, 3.842, 0.77), "Concrete", 0.025)
    for tile in range(4):
        cy = -1.44 + tile * 0.96
        box(item, name + "_DeckSlab_" + str(tile), (0, cy, -0.018), (length - 0.016, 0.95, 0.036), "Concrete", 0.005)
    for edge in (-1, 1):
        box(item, name + "_Fascia_" + str(edge), (0, edge * 1.907, -0.675), (length - 0.012, 0.036, 0.22), "Graphite", 0.008)
    box(item, name + "_EdgeStripe", (0, -1.743, -0.0025), (length - 0.035, 0.105, 0.005), "SafetyOchre", 0.001)
    box(item, name + "_EdgeGroove", (0, -1.858, -0.006), (length - 0.025, 0.028, 0.006), "DarkRecess", 0.001)
    # Recessed expansion marks are separate dark inlays, all within the box.
    for cx in (-length * 0.5 + 0.035, length * 0.5 - 0.035):
        box(item, name + "_EndSeam", (cx, 0, -0.007), (0.012, 3.77, 0.01), "ConcreteEdge", 0.001)
    return item


panel = platform()
end_panel = platform(1.0, "SM_StationPlatformEndPanel_1m")
roof = asset("SM_StationRoofPanel", [[-1.5, -5.65, -0.18], [1.5, 5.65, 0.18]],
             "Canopy midplane centre; place at station up +5.40 m, tile along +X.")
box(roof, "Roof_WeatherSkin", (0, 0, 0.12), (2.992, 11.292, 0.12), "Graphite", 0.015)
for edge in (-1, 1):
    box(roof, "Roof_EdgeFascia", (0, edge * 5.59, -0.008), (2.992, 0.12, 0.332), "BlueSteel", 0.014)
for cx in (-1.425, 0, 1.425):
    box(roof, "Roof_TransverseRib", (cx, 0, -0.068), (0.11, 11.18, 0.224), "Graphite", 0.012)
for cy in (-4.7, -2.8, 0, 2.8, 4.7):
    box(roof, "Roof_UndersideInfill", (0, cy, -0.045), (2.86, 0.72, 0.032), "BlueSteel", 0.006)
for cy in (-3.28, 3.28):
    box(roof, "Roof_LightHousing", (0, cy, -0.102), (2.65, 0.21, 0.10), "DarkRecess", 0.007)
    box(roof, "Roof_Diffuser", (0, cy, -0.157), (2.52, 0.14, 0.014), "LightDiffuser", 0.004)
post = asset("SM_StationPost", [[-0.2, -0.2, 0], [0.2, 0.2, 5.22]], "Post base centre; full-size 5.22 m post, never stretch bolts or top cap.")
box(post, "Post_BoxSection", (0, 0, 2.61), (0.366, 0.366, 5.20), "Graphite", 0.012)
for cy in (-0.184, 0.184):
    box(post, "Post_RecessChannel", (0, cy, 2.65), (0.09, 0.009, 4.74), "DarkRecess", 0.003)
box(post, "Post_BasePlate", (0, 0, 0.028), (0.4, 0.4, 0.056), "BlueSteel", 0.006)
box(post, "Post_CapPlate", (0, 0, 5.198), (0.4, 0.4, 0.044), "MachinedSteel", 0.004)
for cx in (-0.145, 0.145):
    for cy in (-0.145, 0.145):
        bolt(post, "Post_RecessedAnchor", (cx, cy), 0.055, 0.020, 0.014)

tie = asset("SM_TrackTie", [[-0.07, -0.825, -0.08], [0.07, 0.825, 0.08]], "Tie midpoint. Runtime places origin at rail up -0.19 m; repeat at canonical 3 m ties.")
box(tie, "Tie_UpperFlange", (0, 0, 0.061), (0.14, 1.65, 0.038), "BlueSteel", 0.008)
box(tie, "Tie_LowerFlange", (0, 0, -0.064), (0.13, 1.60, 0.032), "BlueSteel", 0.006)
box(tie, "Tie_SectionWeb", (0, 0, -0.001), (0.050, 1.55, 0.126), "Graphite", 0.006)
for cy in (-0.65, 0.65):
    box(tie, "Tie_RailSeat", (0, cy, 0.024), (0.134, 0.29, 0.032), "MachinedSteel", 0.005)
for cy in (-0.775, -0.49, 0.49, 0.775):
    bolt(tie, "Tie_FlushFastener", (0, cy), 0.079, 0.021, 0.008)

web = asset("REVIEW_TrackRailToSpineWeb", [[-0.09, -0.72, -0.71], [0.09, 0.72, -0.085]],
            "Rail midpoint datum. Outside current tie box; DO NOT integrate before adding and validating static hardware solids.", "REVIEW_ONLY_OUTSIDE_TIE_ENVELOPE")
for side in (-1, 1):
    profile = [(side * 0.08, -0.64), (side * 0.18, -0.55), (side * 0.72, -0.16), (side * 0.70, -0.09), (side * 0.54, -0.12), (side * 0.04, -0.45)]
    verts = [(x, y, z) for x in (-0.046, 0.046) for y, z in profile]
    n = len(profile)
    faces = [tuple(reversed(range(n))), tuple(range(n, 2*n))] + [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    mesh_object(web, "Review_WebPlate", verts, faces, "ReviewCoral")

joint = asset("SM_SupportJoint_R032_L032", [[-0.32, -0.32, 0], [0.32, 0.32, 0.32]],
              "Base centre, local +Z toward member top. Reference straight cylinder radius 0.32 m / length 0.32 m; regenerate per actual radius, do not inflate a solid.")
cylinder(joint, "Joint_Core", (0, 0, 0.16), 0.313, 0.313, 0.30, "Graphite")
cylinder(joint, "Joint_BaseLip", (0, 0, 0.028), 0.32, 0.32, 0.056, "BlueSteel")
cylinder(joint, "Joint_TopLip", (0, 0, 0.292), 0.32, 0.32, 0.056, "BlueSteel")
for n in range(8):
    angle = n * math.pi / 4
    bolt(joint, "Joint_RecessedBolt", (0.224 * math.cos(angle), 0.224 * math.sin(angle)), 0.319, 0.027, 0.016)

cap = asset("SM_FootingCap_R180_H020", [[-1.8, -1.8, 0], [1.8, 1.8, 0.2]],
            "Cap-slice base centre. Reference radius 1.8 m / depth 0.20 m; must sit wholly inside each saved footing, not on top of it.")
cylinder(cap, "FootingCap_Concrete", (0, 0, 0.10), 1.80, 1.77, 0.20, "Concrete", 64)
cylinder(cap, "FootingCap_InsetPlate", (0, 0, 0.190), 0.76, 0.76, 0.018, "Graphite", 48)
for n in range(8):
    angle = n * math.pi / 4
    bolt(cap, "FootingCap_RecessedAnchor", (0.58 * math.cos(angle), 0.58 * math.sin(angle)), 0.199, 0.045, 0.028)

witness = asset("REVIEW_CoordinateWitness", [[-0.5, -1.6, 0], [1.6, 0.5, 1]],
                "One-metre cube centred at (0,0,0.5); ochre nose +X, coral rider-right -Y. Verify single UE handedness conversion.", "REVIEW_ONLY_IMPORT_WITNESS")
box(witness, "Witness_OneMetreCube", (0, 0, 0.5), (1, 1, 1), "Graphite", 0)
box(witness, "Witness_Forward_X", (1.02, 0, 0.15), (1.16, 0.14, 0.18), "SafetyOchre", 0.014)
box(witness, "Witness_RiderRight_MinusY", (0, -1.02, 0.15), (0.14, 1.16, 0.18), "ReviewCoral", 0.014)

# Complete tower comes from one checksummed accepted save: every endpoint, radius,
# footing elevation and oblique attachment is preserved, translated only by base.
reference = SUPPORT_REFERENCE["support"]
origin = Vector(reference["base"])
tower = asset("REVIEW_AdaptiveTower_Saved072", None,
              "Assembly pivot is saved support base. Source world axes retained. Each member pivot is its own base and local +Z follows its actual endpoint.", "REVIEW_ONLY_EXACT_SAVED_MEMBER_ASSEMBLY")
for index, member in enumerate(reference["members"]):
    base = Vector(member["base"]) - origin
    top = Vector(member["top"]) - origin
    delta = top - base
    length = delta.length
    rb, rt = member["radiusBase"], member["radiusTop"]
    if length < 0.01 or length > 1000 or min(rb, rt) <= 0 or max(rb, rt) > 5:
        raise ValueError("Invalid saved support member")
    obj = cylinder(tower, "Tower_Member_" + str(index).zfill(3), (0, 0, length/2), rb, rt, length,
                   "Concrete" if member["kind"] == 1 else "BlueSteel", 32)
    obj.location = base
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = delta.to_track_quat("Z", "Y")
    obj["source_member_index"] = index
    obj["radius_base_m"] = rb
    obj["radius_top_m"] = rt
    obj["member_length_m"] = length
    obj["spine_contact"] = member["spineContact"]
    obj["source_base"] = member["base"]
    obj["source_top"] = member["top"]


def instance(item, name, position, collection=stage, rotation_z=0):
    obj = owned(bpy.data.objects.new(OWNER + "_" + name, None))
    obj.instance_type = "COLLECTION"
    obj.instance_collection = item["collection"]
    obj.location = position
    obj.rotation_euler.z = rotation_z
    collection.objects.link(obj)
    return obj


# Exact default station boxes, shown as a separate review assembly. These are NOT
# a saved-building import: runtime must place modules from its own station parts.
for side in (-1, 1):
    start = -18.0
    for index in range(28):
        length = min(3.0, 64.0 - start)
        item = panel if length == 3 else end_panel
        instance(item, "Station_Platform_" + str(side) + "_" + str(index), (start + length/2, side*3.275, 0), rotation_z=0 if side == 1 else math.pi)
        start += length
for index in range(28):
    instance(roof, "Station_Roof_" + str(index), (-19 + index*3 + 1.5, 0, 5.40))
for cx in (-14, 0.8, 15.6, 30.4, 45.2, 60):
    for side in (-1, 1):
        instance(post, "Station_Post", (cx, side*4.80, 0))
# No new stairs, railings or piers outside the validated boxes are invented.
for item, pos in ((panel, (6, -12, 0)), (roof, (12, -14, 2)), (post, (1, -11, 0)),
                  (tie, (4, -17, 0.5)), (web, (6, -17, 1)), (joint, (8, -17, 0)),
                  (cap, (11, -19, 0)), (witness, (15, -19, 0))):
    instance(item, "Gallery_" + item["name"], pos)


def camera(target_scene, name, location, target, lens=48):
    data = owned(bpy.data.cameras.new(OWNER + "_" + name))
    data.lens = lens
    data.clip_end = 2000
    obj = owned(bpy.data.objects.new(OWNER + "_" + name, data))
    target_scene.collection.objects.link(obj)
    obj.location = location
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()
    return obj


def lights(target_scene, extent=30):
    for name, position, energy, size in (("Key", (10, -20, 35), 25000, extent), ("Fill", (-15, 12, 18), 17000, extent)):
        data = owned(bpy.data.lights.new(OWNER + "_" + target_scene.name + name, "AREA"))
        data.energy = energy
        data.shape = "DISK"
        data.size = size
        obj = owned(bpy.data.objects.new(OWNER + "_" + target_scene.name + name, data))
        target_scene.collection.objects.link(obj)
        obj.location = position
        obj.rotation_euler = (-obj.location).to_track_quat("-Z", "Y").to_euler()
    sun = owned(bpy.data.lights.new(OWNER + "_" + target_scene.name + "Sun", "SUN"))
    sun.energy = 2.2
    sun.angle = math.radians(12)
    obj = owned(bpy.data.objects.new(OWNER + "_" + target_scene.name + "Sun", sun))
    target_scene.collection.objects.link(obj)
    obj.rotation_euler = (math.radians(28), math.radians(-22), math.radians(-35))


lights(scene)
scene.camera = camera(scene, "StationCamera", (85, -83, 48), (23, -1, 2), 44)
camera(scene, "ModulesCamera", (24, -34, 16), (8, -13, 1.3), 48)
camera(scene, "StationHumanScaleCamera", (-9, -3.1, 1.7), (17, -3.1, 2.5), 24)
camera(scene, "TieAndWebCamera", (5.2, -19.1, 1.55), (5, -17, 0.63), 55)
scene.render.filepath = EXPORT_DIR + "review_station.png"
tower_scene = make_scene("AdaptiveTowerReview")
tower_stage = owned(bpy.data.collections.new(OWNER + "_TowerStage"))
tower_scene.collection.children.link(tower_stage)
instance(tower, "SavedTower_Assembly", (0, 0, 0), tower_stage)
lights(tower_scene, 60)
tower_scene.camera = camera(tower_scene, "TowerCamera", (115, -125, 93), (0, 0, 35), 52)
tower_scene.render.filepath = EXPORT_DIR + "review_adaptive_tower.png"

# Verify the authored solid fit before exporting. This is a vertex/convex-envelope
# art check, not a replacement for clearance or structural validation.
manifest = {"schema": 1, "kit": OWNER, "units": "metres", "axes": "+X forward, +Z up, -Y rider-right",
            "sourceBlend": SOURCE_BLEND, "runtimeIntegrated": False,
            "supportReference": {key: value for key, value in SUPPORT_REFERENCE.items() if key != "support"},
            "supportIndex": reference["index"], "supportMemberCount": len(reference["members"]), "assets": [],
            "limitations": ["No structural engineering certification.", "Rail/spine paths remain canonical procedural geometry.",
                            "Review web extends outside the tie envelope and cannot be integrated yet.",
                            "Footing/joint references require regenerated dimensions per actual saved solid; do not stretch details.",
                            "Default station assembly is a geometry preview; no invented stairs or ground texture.",
                            "UE import scale/handedness requires the asymmetric witness and 1 m cube test.",
                            "Adaptive tower member rotations encode local +Z along each saved member; object scales are one."]}
export_scene = make_scene("ExportMasters")
for item in assets:
    export_scene.collection.children.link(item["collection"])
bpy.context.window.scene = scene
bpy.context.view_layer.update()
# Save source before native exporters; a later exporter crash must not lose the models.
bpy.ops.wm.save_as_mainfile(filepath=SOURCE_BLEND, check_existing=False, copy=True)
bpy.context.window.scene = export_scene
bpy.context.view_layer.update()
for item in assets:
    vertices = [(obj.rotation_quaternion if obj.rotation_mode == "QUATERNION" else obj.rotation_euler.to_quaternion()) @ v.co + obj.location for obj in item["objects"] for v in obj.data.vertices]
    actual_min = [min(v[axis] for v in vertices) for axis in range(3)]
    actual_max = [max(v[axis] for v in vertices) for axis in range(3)]
    if item["bounds"] is not None:
        low, high = item["bounds"]
        if any(actual_min[axis] < low[axis] - 1e-5 or actual_max[axis] > high[axis] + 1e-5 for axis in range(3)):
            raise ValueError("Asset escaped its envelope: " + item["name"])
    if item == joint or item == cap:
        limit = 0.32 if item == joint else 1.8
        if any(math.hypot(v.x, v.y) > limit + 1e-6 for v in vertices):
            raise ValueError("Circular support detail escaped the reference solid")
    for obj in bpy.context.selected_objects:
        obj.select_set(False)
    for obj in list(item["collection"].objects):
        obj.select_set(True)
    bpy.context.view_layer.objects.active = item["root"]
    glb_path = EXPORT_DIR + item["name"] + ".glb"
    fbx_path = EXPORT_DIR + item["name"] + ".fbx"
    bpy.ops.export_scene.gltf(filepath=glb_path, export_format="GLB", use_selection=True,
                              export_apply=True, export_yup=True, export_animations=False, export_extras=True)
    bpy.ops.export_scene.fbx(filepath=fbx_path, use_selection=True, global_scale=1.0,
                             apply_unit_scale=True, apply_scale_options="FBX_SCALE_UNITS", axis_forward="X", axis_up="Z",
                             use_mesh_modifiers=True, add_leaf_bones=False, bake_anim=False, use_custom_props=True,
                             object_types={"EMPTY", "MESH"}, path_mode="AUTO")
    manifest["assets"].append({"name": item["name"], "status": item["status"], "pivot": item["pivot"],
        "boundsMinimumM": actual_min, "boundsMaximumM": actual_max, "declaredEnvelopeM": item["bounds"],
        "partCount": len(item["objects"]), "vertexCount": sum(len(o.data.vertices) for o in item["objects"]),
        "materialSlots": sorted(set(slot.name for o in item["objects"] for slot in o.data.materials)),
        "glb": glb_path, "fbx": fbx_path})
bpy.context.window.scene = scene
bpy.context.view_layer.update()
if previous_scene is not None:
    bpy.context.window.scene = previous_scene
print("VC_ENVKIT_MANIFEST_BEGIN")
print(json.dumps(manifest, indent=2))
print("VC_ENVKIT_MANIFEST_END")
