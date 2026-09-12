import tempfile
import unittest
import zipfile
from pathlib import Path

from distribute_windows import distribute


class DistributionTests(unittest.TestCase):
    def setUp(self):
        workspace = Path(__file__).resolve().parents[1] / 'unreal/Saved/DistributionTests'
        workspace.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=workspace)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.package = self.root / 'packaged'
        self.files = [
            'VibeCoaster.exe', 'VibeCoaster/Binaries/Win64/VibeCoaster.exe',
            'VibeCoaster/Content/Paks/global.utoc', 'VibeCoaster/Content/Paks/global.ucas',
            'VibeCoaster/Content/Paks/VibeCoaster-Windows.utoc',
            'VibeCoaster/Content/Paks/VibeCoaster-Windows.ucas',
            'VibeCoaster/Content/Paks/VibeCoaster-Windows.pak',
            'NOTICES.txt', 'Engine/Extras/Redist/en-us/vc_redist.x64.exe',
            'VibeCoaster/Binaries/Win64/VibeCoaster.pdb',
        ]
        for name in self.files:
            path = self.package / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(('SYNTHETIC distribution fixture: ' + name).encode())

    def test_missing_cooked_payload_fails_before_creating_output(self):
        for name in ['global.ucas', 'VibeCoaster-Windows.pak']:
            with self.subTest(name=name):
                path = self.package / 'VibeCoaster/Content/Paks' / name
                data = path.read_bytes()
                path.unlink()
                with self.assertRaisesRegex(ValueError, 'Incomplete package'):
                    distribute(self.package, self.root / name, 'test')
                self.assertFalse((self.root / name).exists())
                path.write_bytes(data)

    def test_archive_preserves_runtime_and_prior_delivery(self):
        result = distribute(self.package, self.root / 'output', 'test')
        archive = Path(result['archive'])
        before = archive.read_bytes()
        with zipfile.ZipFile(archive) as zipped:
            for name in self.files:
                key = 'VibeCoaster-test-Windows/' + name
                if name.endswith('.pdb'):
                    self.assertNotIn(key, zipped.namelist())
                else:
                    self.assertEqual(zipped.read(key), (self.package / name).read_bytes())
        with self.assertRaises(FileExistsError):
            distribute(self.package, self.root / 'output', 'test')
        self.assertEqual(archive.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
