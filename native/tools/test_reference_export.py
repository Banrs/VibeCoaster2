"""Synthetic-only tests; no measured benchmark."""
import copy, json, pathlib, tempfile, unittest
import reference_forces as rf
import reference_export as re
from test_reference_forces import good_manifest, write_csv

class ReferenceExportTests(unittest.TestCase):
    def fixture(self, root, n=3):
        files=[]
        for i in range(n):
            csv=root/f"synthetic-{i}.csv"
            write_csv(csv,[(j/10,2+i/10,0,j/1000) for j in range(121)])
            a=rf.analyze_recording(csv,good_manifest(recording_id=f"synthetic-record-{i}"))
            p=root/f"analysis-{i}.json";p.write_text(json.dumps(a));files.append(p)
        return files
    def test_export_three_has_spread_identity_no_uncertainty_claim(self):
        with tempfile.TemporaryDirectory() as t:
            files=self.fixture(pathlib.Path(t)); key=list(rf.group_key(json.loads(files[0].read_text())))
            out=re.export_group(files,key)
            self.assertEqual(out['n_eligible'],3)
            self.assertAlmostEqual(out['median_S_g_s'],21)
            self.assertEqual(out['quartiles_g_s'],[])
            self.assertEqual(out['measurement_uncertainty'],'not-quantified')
            self.assertEqual(len(out['recordings']),3)
            self.assertEqual(out['recordings'][0]['analysis_sha256'],rf.sha256_file(files[0]))
            self.assertTrue(re.interchange(out).startswith('COASTER_REFERENCE 1\n'))
    def test_explicit_group_required(self):
        with tempfile.TemporaryDirectory() as t:
            files=self.fixture(pathlib.Path(t))
            with self.assertRaises(rf.ForceValidationError): re.export_group(files,['wrong']*5)
    def test_raw_changed_and_cached_metric_tamper_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root=pathlib.Path(t);files=self.fixture(root);a=json.loads(files[0].read_text());key=list(rf.group_key(a))
            a['strongest10s']['S_g_s']=999;files[0].write_text(json.dumps(a))
            with self.assertRaises(rf.ForceValidationError): re.export_group(files,key)
            files=self.fixture(root);(root/'synthetic-0.csv').write_text('changed')
            with self.assertRaises(rf.ForceValidationError): re.export_group(files,key)
    def test_duplicate_does_not_create_independent_record(self):
        with tempfile.TemporaryDirectory() as t:
            files=self.fixture(pathlib.Path(t),2);key=list(rf.group_key(json.loads(files[0].read_text())))
            with self.assertRaises(rf.ForceValidationError): re.export_group(files+[files[0]],key)
    def test_four_records_quartiles_and_control_strings(self):
        with tempfile.TemporaryDirectory() as t:
            files=self.fixture(pathlib.Path(t),4);key=list(rf.group_key(json.loads(files[0].read_text())))
            out=re.export_group(files,key);self.assertEqual(len(out['quartiles_g_s']),3)
            out['recordings'][0]['source']='bad\\nsource'.replace('\\n','\n')
            with self.assertRaises(rf.ForceValidationError):re.interchange(out)
    def test_relative_raw_alias_is_protected(self):
        with tempfile.TemporaryDirectory() as t:
            root=pathlib.Path(t);files=self.fixture(root);a=json.loads(files[0].read_text());key=list(rf.group_key(a))
            a['input_csv']='synthetic-0.csv';files[0].write_text(json.dumps(a))
            raw=root/'synthetic-0.csv';before=raw.read_bytes()
            with self.assertRaises(rf.OutputAliasError):
                re.main(['--inputs',*[str(x) for x in files],'--group',json.dumps(key),'--out',str(raw)])
            self.assertEqual(raw.read_bytes(),before)
    def test_longest_bout_separates_recovery_interval(self):
        times=[0,1,2,3,4,5,6];values=[0,3,0,0,3,3,0]
        self.assertAlmostEqual(rf.longest_threshold_bout(times,values,2),5/3)
        self.assertAlmostEqual(rf.time_above(times,values,2),7/3)
        self.assertEqual(rf.longest_threshold_bout([0,1,2],[2,2,2],2),0)
    def test_irregular_signed_window_and_rates(self):
        # Triangular signed force; maximum 10s mean at interior root, not a knot.
        x=rf.axis_summary([0,10,20],[-5,5,-5])
        self.assertAlmostEqual(x['rolling_mean']['10s']['max_g'],2.5)
        self.assertAlmostEqual(x['max_abs_component_rate_g_s'],1)
        self.assertAlmostEqual(x['rolling_mean']['10s']['min_g'],0)
        self.assertEqual(rf.axis_summary([0,.1],[1,2])['rolling_mean']['1s']['status'],'insufficient-duration')

if __name__=='__main__':unittest.main()
