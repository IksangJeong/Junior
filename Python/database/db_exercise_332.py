"""
데이터베이스 연습 #332
SQLite CRUD 실습
"""
import sqlite3

def setup_332():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_332 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_332_0', 600.96, 'B'), ('item_332_1', 865.08, 'A'), ('item_332_2', 298.93, 'B'), ('item_332_3', 781.38, 'C'), ('item_332_4', 902.12, 'A'), ('item_332_5', 576.24, 'A'), ('item_332_6', 688.46, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_332 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_332()
    cursor = conn.cursor()

    print(f"DB 연습 #332")
    cursor.execute('SELECT * FROM items_332')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_332 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
