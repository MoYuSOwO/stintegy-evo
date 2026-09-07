#!/bin/bash
# 两炉状态一览。用法：./status.sh  或  watch -n 30 ./status.sh
cd "$(dirname "$0")"
show() {
  local name="$1" log="$2" tag="$3" target="$4"
  printf '\n\033[1m%s\033[0m  ' "$name"
  if pgrep -f "tag $tag" >/dev/null; then
    local et; et=$(ps -o etime= -p "$(pgrep -f "tag $tag" | head -1)" | tr -d ' ')
    printf '运行中 %s' "$et"
  elif grep -q "training finished\|training stopped" "$log" 2>/dev/null; then
    printf '已结束'
  else
    printf '排队中'
  fi
  [ -f "$log" ] || { printf '（尚无日志）\n'; return; }
  local step start
  step=$(grep -oE '^step +[0-9]+' "$log" | tail -1 | grep -oE '[0-9]+')
  start=$(grep -oE 'step, [0-9]+' "$log" | head -1 | grep -oE '[0-9]+')
  [ -z "$start" ] && start=$(grep -oE 'format 2, step [0-9]+' "$log" | head -1 | grep -oE '[0-9]+$')
  [ -z "$start" ] && start=0
  [ -n "$step" ] && printf '   步 %s / %s（本炉 +%s）' "$step" "$target" "$((step - start))"
  printf '\n'
  printf '  最近旋转/千步: '
  grep -oE 'spins [0-9]+' "$log" | tail -12 | awk '{printf "%s ", $2}'
  printf '\n'
  local a; a=$(grep -oE 'alpha [0-9.]+' "$log" | tail -1 | awk '{print $2}')
  [ -n "$a" ] && printf '  alpha %s' "$a"
  grep -q "alpha frozen" "$log" && printf '  \033[32m[已冻结: %s]\033[0m' "$(grep 'alpha frozen' "$log" | tail -1 | grep -oE 'at [0-9.]+' | awk '{print $2}')"
  printf '\n'
  echo "  --- 评估 ---"
  grep -E '专家 silverstone' "$log" | tail -4 | sed -E 's/ +/ /g; s/^ */    /'
  grep -E '干净口径|saved best|无进步|training stopped' "$log" | tail -3 | sed -E 's/^ */    /'
}
show "臂A 接力 (armA)"  run-armA-relay.log   armA 725000
show "臂B 认证 (armB)"  run-armB-scratch.log armB 400000
echo
