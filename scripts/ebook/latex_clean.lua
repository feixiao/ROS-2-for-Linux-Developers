-- ==============================================================================
-- Pandoc Lua Filter: Clean Unicode & Emojis for LaTeX Compilation
-- ==============================================================================

local emoji_replacements = {
  ["❌"] = "[×]",
  ["⚠️"] = "[!]",
  ["⚠"]  = "[!]",
  ["✅"] = "[√]",
  ["🚀"] = ">",
  ["📖"] = "*",
  ["🛠️"] = "*",
  ["🛠"]  = "*",
  ["⚙️"] = "*",
  ["⚙"]  = "*",
  ["💡"] = "[提示]",
  ["🔍"] = "[分析]",
  ["📌"] = "[重点]"
}

function Str(el)
  if FORMAT:match("latex") then
    local text = el.text
    -- Remove variation selector-16 (U+FE0F)
    text = text:gsub("\239\184\143", "")
    for emoji, replacement in pairs(emoji_replacements) do
      if text:find(emoji, 1, true) then
        text = text:gsub(emoji, replacement)
      end
    end
    el.text = text
  end
  return el
end
