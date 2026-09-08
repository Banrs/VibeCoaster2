import pathlib, unittest
BASE=pathlib.Path(__file__).resolve().parents[1]/"unreal/Source/VibeCoaster/Private"
class UIContract(unittest.TestCase):
    def test_history_only_updates_in_revision_checked_commit(self):
        s=(BASE/"VibeCoasterWorld.cpp").read_text();self.assertEqual(s.count("Comparisons.commit"),1)
        commit=s.split("void AVibeCoasterWorld::CommitChunks()",1)[1].split("void AVibeCoasterWorld::Tick",1)[0]
        self.assertLess(commit.index("Prepared.Revision != Runtime->Revision"),commit.index("Comparisons.commit"))
        self.assertLess(commit.index("Retire(Active)"),commit.index("Comparisons.commit"))
    def test_explicit_file_does_not_fallback_and_compare_input_is_separate(self):
        s=(BASE/"VibeCoasterGame.cpp").read_text()
        self.assertIn("coaster::loadReference",s);self.assertLess(s.index("return; // An explicitly requested file"),s.index("double Exposure = 0"))
        self.assertIn("if (!Menu || ShowComparison) return;",s)
        self.assertIn("ComparisonOffset = FMath::Clamp",s)
        self.assertIn("EKeys::C",s)
