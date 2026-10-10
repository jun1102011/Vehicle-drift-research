# 仅清理本批已逐字节重建的绘图缓存、编译缓存及与存档哈希一致的运行重复证据。
$ErrorActionPreference='Stop'
$batchRoot=[IO.Path]::GetFullPath((Split-Path -Parent $PSScriptRoot))
$runtimeRoot=[IO.Path]::GetFullPath('C:\Users\28583\AppData\Local\Temp\step37c_road_coordinate_20261010')
$audit=Get-Content -LiteralPath (Join-Path $batchRoot 'audit\final_integrity.json') -Raw | ConvertFrom-Json
if(-not ($audit.passed -and $audit.cache_bitwise_rebuilt -and $audit.metrics_exact_rebuilt)){throw '缓存复核门槛未通过'}
$deleted=@()
foreach($item in $audit.raw){
 $rp=Join-Path $batchRoot $item.path
 if((Get-FileHash -LiteralPath $rp -Algorithm SHA256).Hash.ToLowerInvariant() -ne $item.sha256){throw '原始数据哈希改变'}
}
foreach($property in $audit.cache.PSObject.Properties){
 $cp=[IO.Path]::GetFullPath((Join-Path $batchRoot $property.Name))
 if(-not $cp.StartsWith($batchRoot+'\cache\',[StringComparison]::OrdinalIgnoreCase)){throw '缓存路径越界'}
 if((Get-FileHash -LiteralPath $cp -Algorithm SHA256).Hash.ToLowerInvariant() -ne $property.Value.sha256){throw '缓存重建后又发生变化'}
}
$protocol=Get-Content -LiteralPath (Join-Path $batchRoot 'source\protocol.json') -Raw | ConvertFrom-Json
foreach($case in $protocol.cases){
 $run=Join-Path $batchRoot ('results\runs\LeftRight_R3_'+$case)
 $ev=Get-Content -LiteralPath (Join-Path $run 'CarSim_evidence_index.json') -Raw | ConvertFrom-Json
 foreach($prop in $ev.outputs.PSObject.Properties){
  $item=$prop.Value;$rp=[IO.Path]::GetFullPath($item.source);$saved=[IO.Path]::GetFullPath($item.saved)
  if(-not $rp.StartsWith($runtimeRoot+'\',[StringComparison]::OrdinalIgnoreCase)){throw '运行副本路径越界'}
  if(-not $saved.StartsWith($batchRoot+'\results\runs\',[StringComparison]::OrdinalIgnoreCase)){throw '存档路径越界'}
  if(-not (Test-Path -LiteralPath $rp)){throw '重复副本缺失，不能重复清理'}
  if((Get-FileHash -LiteralPath $rp).Hash -ne (Get-FileHash -LiteralPath $saved).Hash){throw '证据副本不一致'}
  $deleted+=@{path=$rp;bytes=(Get-Item -LiteralPath $rp).Length;sha256=(Get-FileHash -LiteralPath $rp).Hash.ToLowerInvariant();verified_copy=$saved}
 }
}
# 所有证据先验证完，再删除；sim/par不在候选列表中。
foreach($item in $deleted){Remove-Item -LiteralPath $item.path -Force}
$cacheDeleted=@()
foreach($relative in @('cache','model\slprj')){
 $target=[IO.Path]::GetFullPath((Join-Path $batchRoot $relative))
 if(-not $target.StartsWith($batchRoot+'\',[StringComparison]::OrdinalIgnoreCase)){throw '递归清理路径越界'}
 if(Test-Path -LiteralPath $target){
  foreach($file in (Get-ChildItem -LiteralPath $target -Recurse -File)){$cacheDeleted+=@{path=$file.FullName;bytes=$file.Length}}
  Remove-Item -LiteralPath $target -Recurse -Force
 }
}
foreach($item in $audit.raw){if((Get-FileHash -LiteralPath (Join-Path $batchRoot $item.path)).Hash.ToLowerInvariant() -ne $item.sha256){throw '清理后raw改变'}}
$record=@{passed=$true;raw_unchanged=$true;runtime_duplicates=$deleted;derived_cache=$cacheDeleted;configuration_files_preserved=$true}
$record | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $batchRoot 'audit\verified_cleanup.json') -Encoding UTF8
Write-Output ('CLEANUP_PASS runtime_files='+$deleted.Count+' cache_files='+$cacheDeleted.Count)
