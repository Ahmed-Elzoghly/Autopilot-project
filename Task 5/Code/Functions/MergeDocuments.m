function MergeDocuments(PT,OutFile)
Word_COM = actxserver('Word.Application');
set(Word_COM,'visible',0);
My_Report = invoke(Word_COM.documents,'add');
My_Report.PageSetup.Orientation = 'wdOrientPortrait';
ReportDoc = Word_COM.Selection;
ReportDoc.PageSetup.TopMargin = 30;
ReportDoc.PageSetup.LeftMargin = 30;
ReportDoc.PageSetup.RightMargin = 30;
ReportDoc.PageSetup.BottomMargin = 30;
LPT = length(PT);
for i = 1:LPT
ReportDoc.InsertFile([pwd '\' PT{i}]);
end
My_Report.SaveAs2([pwd '\' OutFile],17)
My_Report.Close(0);
invoke(Word_COM,'Quit');
delete(Word_COM);
end